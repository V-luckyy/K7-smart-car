// 移植自 wheeltec_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp (ROS2 Humble)
// Ported from wheeltec_robot.cpp (ROS2 Humble). Changes:
// 1. include 路径改为 k7_bringup/k7_robot.h、k7_bringup/Quaternion_Solution.h；
//    删除 wheeltec_robot_msg/msg/data.hpp include（K7 工作区无此消息包，且该 include 本就未被使用）
// 2. 类名/节点名 turn_on_robot/wheeltec_robot -> K7SerialNode/k7_serial_node
// 3. usart_port_name 参数默认值 /dev/wheeltec_controller -> /dev/k7_controller（参数名不变）
// 4. 新增 cmd_vel 看门狗（K7 固件断链不停车，必须软件兜底）：
//    - 新参数 cmd_vel_timeout_ms，默认 500
//    - 原 Cmd_Vel_Callback 的组帧/发送代码抽为 Send_Cmd_Vel()，Cmd_Vel_Callback 仅更新时间戳后调用之
//    - 100ms 周期 wall timer 调用 Cmd_Vel_Watchdog_Check()，超时即复用同一代码路径发零速度帧(10Hz)；
//      节点启动后尚未收到任何 cmd_vel 时 last_cmd_vel_time_ 取启动时刻，超时后同样发零速(无害)
// 5. 日志字符串中的节点名相应改为 k7_serial_node
// 其余（协议解析、里程计积分、回充帧处理、协方差矩阵、串口参数）与 wheeltec 原文件逐行一致，未做任何"顺手优化"

#include "k7_bringup/k7_robot.h"
#include "k7_bringup/Quaternion_Solution.h"

sensor_msgs::msg::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 

using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

//自动回充使用相关变量
bool check_AutoCharge_data = false;
bool charge_set_state = false;
bool check_ranger_data = false; //红外测距帧接收成功标志位
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过K7SerialNode类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
	K7SerialNode Robot_Control;//Instantiate an object //实例化一个对象
	Robot_Control.Control();//Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
    return 0;  
} 

/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/
short K7SerialNode::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;   
  transition_16 |=  Data_Low;
  return transition_16;     
}
float K7SerialNode::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}
/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void K7SerialNode::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  last_cmd_vel_time_ = this->now(); //Record the latest cmd_vel time for the watchdog //记录最近一次cmd_vel时间，供看门狗使用
  Send_Cmd_Vel(twist_aux->linear.x, twist_aux->linear.y, twist_aux->angular.z); //Frame and send to the lower computer //组帧并发送给下位机
}
/**************************************
Date: January 28, 2021
Function: Frame the velocity command and send it to the lower computer through the serial port.
          Shared by Cmd_Vel_Callback and the cmd_vel watchdog (zero speed on timeout).
功能: 速度指令组帧并通过串口发给下位机，Cmd_Vel_Callback 与 cmd_vel 看门狗(超时发零速)复用本函数
***************************************/
void K7SerialNode::Send_Cmd_Vel(double linear_x, double linear_y, double angular_z)
{
  short  transition;  //intermediate variable //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = AutoRecharge; //set aside //预留位
  Send_Data.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = linear_x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  transition=0;
  transition = linear_y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = angular_z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}
/**************************************
Function: cmd_vel watchdog check. The K7 firmware has NO command-loss protection (keeps the last
          velocity forever on link loss), so when no cmd_vel has been received within
          cmd_vel_timeout_ms, send a zero speed frame through the same framing path as
          Cmd_Vel_Callback. Checked every 100ms; sends every check while timed out (10Hz).
          Before the first cmd_vel after startup, last_cmd_vel_time_ equals the startup
          time, so zero frames are also sent after the timeout (harmless).
功能: cmd_vel看门狗检查。K7固件无命令丢失保护(断链后保持最后速度)，当超过cmd_vel_timeout_ms
      未收到cmd_vel时，复用与Cmd_Vel_Callback相同的组帧/发送代码路径发送零速度帧。
      每100ms检查一次，超时期间每次检查都发(10Hz零速帧，无害且更鲁棒)。
      节点启动后尚未收到任何cmd_vel时视为超时(发零速，无害)。
***************************************/
void K7SerialNode::Cmd_Vel_Watchdog_Check()
{
  if((this->now() - last_cmd_vel_time_).seconds()*1000.0 > cmd_vel_timeout_ms)
  {
    Send_Cmd_Vel(0.0, 0.0, 0.0); //Send zero speed frame to stop the robot //发送零速度帧使小车停止
  }
}
/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void K7SerialNode::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now(); 
  Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  
  imu_publisher->publish(Imu_Data_Pub); //Pub IMU topic //发布IMU话题
}
/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void K7SerialNode::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = rclcpp::Node::now(); ; 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
}
/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void K7SerialNode::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
        voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
      }
}

////////// 回充发布与回调 ////////
/**************************************
Date: August 23, 2026
Function: Publish the 3-way IR distance topic (front / left45 / right45, meters)
功能: 发布三路红外测距话题(前 / 左45° / 右45°，米)
***************************************/
void K7SerialNode::Publish_IrDistances()
{
    k7_msgs::msg::IrDistances msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = robot_frame_id;
    msg.front  = ir_dist_front_;
    msg.left45 = ir_dist_left45_;
    msg.right45 = ir_dist_right45_;
    ir_distances_publisher->publish(msg);
}

/**************************************
Date: January 17, 2022
Function: Pub the topic whether the robot finds the infrared signal (charging station)
功能: 发布机器人是否寻找到红外信号(充电桩)的话题
***************************************/
void K7SerialNode::Publish_RED()
{
    std_msgs::msg::UInt8 msg;
    msg.data=Red;
    RED_publisher->publish(msg); 

}
/**************************************
Date: January 14, 2022
Function: Publish a topic about whether the robot is charging
功能: 发布机器人是否在充电的话题
***************************************/
void K7SerialNode::Publish_Charging()
{
    static bool last_charging;
    std_msgs::msg::Bool msg;
    msg.data=Charging;
    Charging_publisher->publish(msg); 
    if(last_charging==false && Charging==true)cout<<GREEN<<"Robot is charging."<<endl<<RESET;
    if(last_charging==true && Charging==false)cout<<RED  <<"Robot charging has disconnected."<<endl<<RESET;
    last_charging=Charging;
}
/**************************************
Date: January 28, 2021
Function: Publish charging current information
功能: 发布充电电流信息
***************************************/
void K7SerialNode::Publish_ChargingCurrent()
{
    std_msgs::msg::Float32 msg; 
    msg.data=Charging_Current;
    Charging_current_publisher->publish(msg);
}

/**************************************
Date: March 1, 2022
Function: Infrared connection speed topic subscription Callback function, according to the subscription command through the serial port to set the infrared connection speed
功能: 红外对接速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令设置红外对接速度
***************************************/
void K7SerialNode::Red_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short  transition;  //intermediate variable //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B

  Send_Data.tx[1] = 3; //Infrared docking speed setting flag bit = 3 //红外对接速度设置标志位=3
  Send_Data.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux->linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux->angular.z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);  //BCC check //BCC校验
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 14, 2022
Function: Subscription robot recharge flag bit topic, used to tell the lower machine speed command is normal command or recharge command
功能: 订阅机器人是否回充标志位话题，用于告诉下位机速度命令是正常命令还是回充命令
***************************************/
void K7SerialNode::Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag)
{
  AutoRecharge=Recharge_Flag->data;
}

//服务
void K7SerialNode::Set_Charge_Callback(const shared_ptr<turtlesim::srv::Spawn::Request> req,shared_ptr<turtlesim::srv::Spawn::Response> res)
{
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B

    if(round(req->x)==1)
      Send_Data.tx[1] = 1;
    else if(round(req->x)==2)
      Send_Data.tx[1] = 2; 
    else if(round(req->x)==0)
      Send_Data.tx[1] = 0,AutoRecharge=0;

    Send_Data.tx[2] = 0; 
    Send_Data.tx[3] = 0;  
    Send_Data.tx[4] = 0;   
    Send_Data.tx[5] = 0;
    Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    try
    {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    }
    catch (serial::IOException& e)   
    {
      res->name = "false";
    }

    if( Send_Data.tx[1]==0 )
    {
      if(charge_set_state==0)
        AutoRecharge=0,res->name = "true";
      else
        res->name = "false";
    }
    else
    {
      if(charge_set_state==1)
        res->name = "true";
      else
        res->name = "false";
    }
    return;
}
////////// 回充发布与回调 ////////

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char K7SerialNode::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

//自动回充专用校验位
unsigned char K7SerialNode::Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_AutoCharge_Data.rx[k]; //By bit or by bit //按位异或
    }
  }

  return check_sum;
}

/**************************************
Date: November 18, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
Update Note: This checking method can lead to read error data or correct data not to be processed. Instead of this checking method, frame-by-frame checking is now used. 
             Refer to Get_ Sensor_ Data_ New() function
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
更新说明：该校验方法会导致出现读取错误数据或者正确数据不处理的情况，现在已不用该校验方法，换成逐帧校验方式，参考Get_Sensor_Data_New()函数
***************************************/
bool K7SerialNode::Get_Sensor_Data()
{ 
  short transition_16=0, j=0, Header_Pos=0, Tail_Pos=0; //Intermediate variable //中间变量
  static int flag_error=0,temp=1; //Static variable that records the error flag and location //静态变量，用于记录出错标志位和出错位置
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0},Receive_Data_Tr[temp]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  if(flag_error==0) //Normal condition detected //检测到正常情况
    Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据
  else if (flag_error==1) //Error condition detected 检测到错误情况
  {
    //Read wrong bit data through serial port, read only and do not process, so that the correct data is read next time
    //通过串口读取错位数据，只读取不处理，以便于下次读取到的是正确的数据
    Stm32_Serial.read(Receive_Data_Tr,sizeof (Receive_Data_Tr)); 
    flag_error=0; //Error flag position 0 //错误标志位置0
  }

  /*//View the received raw data directly and debug it for use//直接查看接收到的原始数据，调试使用
  ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
  Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],Receive_Data_Pr[7],
  Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13],Receive_Data_Pr[14],Receive_Data_Pr[15],
  Receive_Data_Pr[16],Receive_Data_Pr[17],Receive_Data_Pr[18],Receive_Data_Pr[19],Receive_Data_Pr[20],Receive_Data_Pr[21],Receive_Data_Pr[22],Receive_Data_Pr[23]);
  */  

  //Record the position of the head and tail of the frame //记录帧头帧尾位置
  for(j=0;j<24;j++)
  {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
  }

  if(Tail_Pos==(Header_Pos+23))
  {
    //If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
    //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
    // ROS_INFO("1-----");
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    flag_error=0; //Error flag position 0 for next reading //错误标志位置0，便于下次读取
  }
  else if(Header_Pos==(1+Tail_Pos))
  {
    //If the header is behind the end of the frame, record the position of the header so that the next reading of the error bit data can correct the data position
    //如果帧头在帧尾后面，记录帧头出现的位置，便于下次读取出错位数据以纠正数据位置
    //|********7D (7B************|**********7D) 7B************|
    // ROS_INFO("2-----");
    temp=Header_Pos; //Record the length of the next read, calculated to be exactly the position of the frame head //记录下一次读取的长度，经计算正好为帧头的位置
    flag_error=1; //Error flag position 1, error bit array for next read //错误标志位置1，让下一次读取出错位数组
    return false;
  }
  else 
  {
    ////其它情况则认为数据包有错误，这种情况一般是正常的数据，但是除帧头帧尾在数据中间出现了7B或7D的数据
    // In other cases, the packet is considered to be faulty
    // This is generally normal data, but there is 7B or 7D data in the middle of the data except for the frame header and end.
    // ROS_INFO("3-----");
    return false;
  }    
  /* //Check receive_data.rx for debugging use //查看Receive_Data.rx，调试使用
  ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
  Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
  Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
  Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]); 
  */
  Receive_Data.Frame_Header= Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail= Receive_Data.rx[23];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER ) //Judge the frame header //判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //Judge the end of the frame //判断帧尾
    { 
      if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)) //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错
      {
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                                                                         //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
        
        //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
        //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        //Get the battery voltage
        //获取电池电压
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;
        transition_16 |=  Receive_Data.rx[21];  
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)

        return true;
     }
    }
  } 
  return false;
}
/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool K7SerialNode::Get_Sensor_Data_New()
{
  short transition_16=0; //Intermediate variable //中间变量
  uint8_t b=0, check=0, k=0, frame_type=0;

  if(Stm32_Serial.read(&b,1)!=1) return false; //读1字节，无数据/超时则返回

  //等帧头：只认 0x7B(主帧)/0xFA(测距帧)/0x7C(回充帧)，其余丢弃（字节级重同步）
  if(parse_state_==0)
  {
    if(b==FRAME_HEADER)          { parse_state_=1; parse_expected_=RECEIVE_DATA_SIZE; }
    else if(b==Distance_HEADER)  { parse_state_=2; parse_expected_=Distance_DATA_size; }
    else if(b==AutoCharge_HEADER){ parse_state_=3; parse_expected_=AutoCharge_DATA_SIZE; }
    else return false; //非帧头，丢弃

    parse_idx_=1;
    parse_buf_[0]=b;
    return false; //帧头已存，等后续字节
  }

  //收集帧体
  parse_buf_[parse_idx_++]=b;
  if(parse_idx_<parse_expected_) return false; //未收满

  //收满一帧：记帧型，复位状态机，再校验分发
  frame_type=parse_state_;
  parse_state_=0;
  parse_expected_=0;
  parse_idx_=0;

  if(frame_type==1) //主帧 24B 0x7B..0x7D
  {
    if(parse_buf_[RECEIVE_DATA_SIZE-1]!=FRAME_TAIL) return false;
    check=0; for(k=0;k<22;k++) check^=parse_buf_[k];
    if(check!=parse_buf_[22]) return false;

    memcpy(Receive_Data.rx,parse_buf_,RECEIVE_DATA_SIZE);
    Receive_Data.Frame_Header=Receive_Data.rx[0];
    Receive_Data.Frame_Tail=Receive_Data.rx[RECEIVE_DATA_SIZE-1];
    Receive_Data.Flag_Stop=Receive_Data.rx[1];

    Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);
    Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);
    Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]);

    Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);
    Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);
    Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);
    Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);
    Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);
    Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);

    Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
    Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
    Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
    Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

    transition_16 = 0;
    transition_16 |=  Receive_Data.rx[20]<<8;
    transition_16 |=  Receive_Data.rx[21];
    Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;

    return true;
  }

  if(frame_type==2) //测距帧 19B 0xFA..0xFC
  {
    if(parse_buf_[Distance_DATA_size-1]!=Distance_TAIL) return false;
    check=0; for(k=0;k<17;k++) check^=parse_buf_[k];
    if(check!=parse_buf_[17]) return false;

    //rangerA=front[1:2], rangerB=left45[3:4], rangerC=right45[5:6]，int16 大端 mm→m
    ir_dist_front_  = (float)(short)((parse_buf_[1]<<8)|parse_buf_[2]) / 1000.0f;
    ir_dist_left45_ = (float)(short)((parse_buf_[3]<<8)|parse_buf_[4]) / 1000.0f;
    ir_dist_right45_= (float)(short)((parse_buf_[5]<<8)|parse_buf_[6]) / 1000.0f;

    check_ranger_data = true; //由 Control() 发布 /ir_distances
    return false;
  }

  if(frame_type==3) //回充帧 8B 0x7C..0x7F
  {
    if(parse_buf_[AutoCharge_DATA_SIZE-1]!=AutoCharge_TAIL) return false;
    check=0; for(k=0;k<6;k++) check^=parse_buf_[k];
    if(check!=parse_buf_[6]) return false;

    transition_16 = 0;
    transition_16 |=  parse_buf_[1]<<8;
    transition_16 |=  parse_buf_[2];
    Charging_Current = transition_16/1000+(transition_16 % 1000)*0.001;
    Red =  parse_buf_[3];
    Charging = parse_buf_[4];
    charge_set_state = parse_buf_[5];
    check_AutoCharge_data = true;

    return false;
  }

  return false;
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void K7SerialNode::Control()
{
  //_Last_Time = ros::Time::now();
  _Last_Time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
  	try
  	{
    //_Now = ros::Time::now();
    _Now = rclcpp::Node::now();
    Sampling_Time = (_Now - _Last_Time).seconds();  //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 
                                                 //获取时间间隔，用于积分速度获得位移(里程) 
    if (true == Get_Sensor_Data_New()) //The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
                                   //通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 

      //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
      //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

      Publish_Odom();      //Pub the speedometer topic //发布里程计话题
      Publish_ImuSensor(); //Pub the IMU topic //发布IMU话题    
      Publish_Voltage();   //Pub the topic of power supply voltage //发布电源电压话题

      _Last_Time = _Now; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
      
    }
    
    //自动回充数据话题
    if(check_AutoCharge_data)
    {
      Publish_Charging();  //Pub a topic about whether the robot is charging //发布机器人是否在充电的话题
      Publish_RED();       //Pub the topic whether the robot finds the infrared signal (charging station) //发布机器人是否寻找到红外信号(充电桩)的话题
      Publish_ChargingCurrent(); //Pub the charging current topic //发布充电电流话题
      check_AutoCharge_data = false;
    }

    //红外测距话题
    if(check_ranger_data)
    {
      Publish_IrDistances();
      check_ranger_data = false;
    }

    rclcpp::spin_some(this->get_node_base_interface());   //The loop waits for the callback function //循环等待回调函数
    }
    
    catch (const rclcpp::exceptions::RCLError & e )
  {
	RCLCPP_ERROR(this->get_logger(),"unexpectedly failed whith %s",e.what());	
	}
}
}
/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
K7SerialNode::K7SerialNode():rclcpp::Node ("k7_serial_node")
{
  Sampling_Time=0;
  Power_voltage=0;
  //Clear the data
  //清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  //ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  
  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/k7_controller"); //K7: fixed by udev rule //K7：由udev规则固定的串口设备名
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<int>("cmd_vel_timeout_ms", 500); //cmd_vel watchdog timeout //cmd_vel看门狗超时时间

  this->get_parameter("serial_baud_rate", serial_baud_rate);//Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  this->get_parameter("usart_port_name", usart_port_name);//Fixed serial port number //固定串口号
  this->get_parameter("odom_frame_id", odom_frame_id);//The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  this->get_parameter("robot_frame_id", robot_frame_id);//The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  this->get_parameter("gyro_frame_id", gyro_frame_id);//IMU topics correspond to TF coordinates //IMU话题对应TF坐标
  this->get_parameter("cmd_vel_timeout_ms", cmd_vel_timeout_ms);//cmd_vel watchdog timeout in ms //cmd_vel看门狗超时时间，单位ms

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);//Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2); //Create an IMU topic publisher //创建IMU话题发布者
  ir_distances_publisher = create_publisher<k7_msgs::msg::IrDistances>("ir_distances", 10); //红外测距单话题发布者
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);//Create a battery-voltage topic publisher //创建电池电压话题发布者

  //回充发布者
  Charging_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_flag", 10);
  Charging_current_publisher = create_publisher<std_msgs::msg::Float32>("robot_charging_current", 10);
  RED_publisher = create_publisher<std_msgs::msg::UInt8>("robot_red_flag", 10);
  //回充订阅者
  Red_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "red_vel", 10, std::bind(&K7SerialNode::Red_Vel_Callback, this, std::placeholders::_1));
  Recharge_Flag_Sub = create_subscription<std_msgs::msg::Int8>(
      "robot_recharge_flag", 10, std::bind(&K7SerialNode::Recharge_Flag_Callback, this,std::placeholders::_1));
  //回充服务提供
  SetCharge_Service=this->create_service<turtlesim::srv::Spawn>
  ("/set_charge",std::bind(&K7SerialNode::Set_Charge_Callback,this,
    std::placeholders::_1 ,std::placeholders::_2));
    
  //Set the velocity control command callback function
  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 2, std::bind(&K7SerialNode::Cmd_Vel_Callback, this, _1));

  //cmd_vel watchdog: send zero speed when cmd_vel is lost (the K7 firmware keeps the last velocity on link loss)
  //cmd_vel看门狗：cmd_vel丢失时发送零速(K7固件断链后保持最后速度，不自动停车)
  last_cmd_vel_time_ = this->now(); //Before the first cmd_vel, timeout is counted from startup //首个cmd_vel之前，从启动时刻起算超时
  cmd_vel_watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&K7SerialNode::Cmd_Vel_Watchdog_Check, this));

  RCLCPP_INFO(this->get_logger(),"k7_serial_node Data ready"); //Prompt message //提示信息

  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"k7_serial_node can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"k7_serial_node serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
K7SerialNode::~K7SerialNode()
{
  //Sends the stop motion command to the lower machine before the K7SerialNode object ends
  //对象K7SerialNode结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口  
  RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
}
