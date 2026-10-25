#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>
#include <memory>
#include <stdexcept>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

using Clock = std::chrono::steady_clock;

class SerialBridge : public rclcpp::Node {
 public:
  SerialBridge() : Node("serial_bridge") {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    const auto device = declare_parameter<std::string>("device", "/dev/k7_controller", startup_only);
    const auto baud = declare_parameter<int>("baud", 115200, startup_only);
    timeout_ = declare_parameter<double>("command_timeout", 0.25, startup_only);
    if (baud != 115200 || !std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 0.3)
      throw std::invalid_argument("invalid firmware baud or command timeout");
    base_ = declare_parameter<std::string>("base_frame", "base_link", startup_only);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom", startup_only);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "gyro_link", startup_only);
    publish_tf_ = declare_parameter<bool>("publish_tf", true, startup_only);
    serial_.setPort(device);
    serial_.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(2);
    serial_.setTimeout(timeout);
    serial_.open();
    serial_.flushInput();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("battery/voltage", 1);
    disabled_pub_ = create_publisher<std_msgs::msg::Bool>("chassis/disabled", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 1,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const double age = (now()-rclcpp::Time(msg->header.stamp)).seconds();
        const auto& t = msg->twist;
        if (msg->header.frame_id != base_ || age < -0.05 || age > timeout_ ||
            !std::isfinite(t.linear.x) || !std::isfinite(t.angular.z)) return;
        v_ = std::clamp(t.linear.x, 0.0, 0.35);
        w_ = std::clamp(t.angular.z, -1.2, 1.2);
        last_command_ = Clock::now();
      });
    read_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { read(); });
    write_timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] {
      const bool fresh = std::chrono::duration<double>(Clock::now()-last_command_).count() <= timeout_;
      send(fresh ? v_ : 0.0, fresh ? w_ : 0.0);
    });
    send(0, 0);
  }

  ~SerialBridge() override {
    try { send(0, 0); } catch (...) {}
  }

 private:
  static uint8_t bcc(const uint8_t* data, size_t length) {
    uint8_t value = 0;
    for (size_t i = 0; i < length; ++i) value ^= data[i];
    return value;
  }
  static int16_t word(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
  }
  void send(double v, double w) {
    uint8_t frame[11] = {0x7b, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x7d};
    const auto linear = static_cast<int16_t>(v*1000);
    const auto angular = static_cast<int16_t>(w*1000);
    frame[3] = static_cast<uint16_t>(linear) >> 8;
    frame[4] = linear & 0xff;
    frame[7] = static_cast<uint16_t>(angular) >> 8;
    frame[8] = angular & 0xff;
    frame[9] = bcc(frame, 9);
    if (serial_.write(frame, 11) != 11) throw std::runtime_error("short UART command write");
  }
  void read() {
    const auto available = std::min<size_t>(serial_.available(), 512);
    if (!available) return;
    std::vector<uint8_t> incoming(available);
    incoming.resize(serial_.read(incoming.data(), available));
    buffer_.insert(buffer_.end(), incoming.begin(), incoming.end());
    while (buffer_.size() >= 24) {
      const auto head = std::find(buffer_.begin(), buffer_.end(), 0x7b);
      buffer_.erase(buffer_.begin(), head);
      if (buffer_.size() < 24) break;
      if (buffer_[23] == 0x7d && buffer_[22] == bcc(buffer_.data(), 22)) {
        publish(buffer_.data());
        buffer_.erase(buffer_.begin(), buffer_.begin()+24);
      } else {
        buffer_.erase(buffer_.begin());
      }
    }
  }
  void publish(const uint8_t* frame) {
    const auto current = Clock::now();
    const double elapsed = last_frame_ == Clock::time_point{} ? 0 :
      std::chrono::duration<double>(current-last_frame_).count();
    last_frame_ = current;
    // After an uplink gap, do not integrate the latest speed across missing history.
    const double dt = elapsed <= 0.15 ? elapsed : 0;
    const double vx = word(frame+2)/1000.0, vy = word(frame+4)/1000.0;
    const double wz = word(frame+6)/1000.0;
    const double mid = theta_+wz*dt/2;
    x_ += (vx*std::cos(mid)-vy*std::sin(mid))*dt;
    y_ += (vx*std::sin(mid)+vy*std::cos(mid))*dt;
    theta_ = std::atan2(std::sin(theta_+wz*dt), std::cos(theta_+wz*dt));
    const auto stamp = now();
    std_msgs::msg::Bool disabled;
    disabled.data = frame[1] != 0;
    disabled_pub_->publish(disabled);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(theta_/2);
    odom.pose.pose.orientation.w = std::cos(theta_/2);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    for (int i : {0, 7, 35}) { odom.pose.covariance[i] = 0.03; odom.twist.covariance[i] = 0.03; }
    for (int i : {14, 21, 28}) { odom.pose.covariance[i] = 1e6; odom.twist.covariance[i] = 1e6; }
    odom_pub_->publish(odom);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation.w = 1;
    imu.orientation_covariance[0] = -1;
    imu.linear_acceleration.x = word(frame+8)/1671.84;
    imu.linear_acceleration.y = word(frame+10)/1671.84;
    imu.linear_acceleration.z = word(frame+12)/1671.84;
    imu.angular_velocity.x = word(frame+14)*0.00026644;
    imu.angular_velocity.y = word(frame+16)*0.00026644;
    imu.angular_velocity.z = word(frame+18)*0.00026644;
    for (int i : {0, 4, 8}) { imu.angular_velocity_covariance[i] = 0.02; imu.linear_acceleration_covariance[i] = 0.1; }
    imu_pub_->publish(imu);
    if (current-last_voltage_ >= std::chrono::seconds(1)) {
      std_msgs::msg::Float32 voltage;
      voltage.data = word(frame+20)/1000.0;
      voltage_pub_->publish(voltage);
      last_voltage_ = current;
    }
  }
  serial::Serial serial_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr disabled_pub_;
  std::string base_, odom_frame_, imu_frame_;
  bool publish_tf_ = true;
  double timeout_ = 0.25, v_ = 0, w_ = 0, x_ = 0, y_ = 0, theta_ = 0;
  Clock::time_point last_command_{}, last_frame_{}, last_voltage_{};
  std::vector<uint8_t> buffer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr read_timer_, write_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<SerialBridge>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
