"""K7 MPC 节点：/odom_combined + 3 路红外 Range → ProgressiveMPC → /cmd_vel_mpc。

闭环逐环节对齐仿真工程 five_version_progressive_r04_with_pid：
- 状态 (x, y, θ) 来自 /odom_combined（EKF 融合里程计）；收到首帧里程计时，
  把硬编码 8 字路径经 SE(2) 刚体变换对齐到小车当前位姿（不动算法本体）。
- 3 路红外 Range → {"front", "left45", "right45"} 距离字典（米）；
  无数据/超量程按 SENSOR_MAX_RANGE 处理（仿真“无检测”语义）。
- 输出 (v, omega) → geometry_msgs/Twist(linear.x=v, angular.z=omega)，
  正 omega = 左转，与仿真一致。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

from . import mpc_config
from .mpc_lib.common_config import DT_CONTROL, FINISH_INDEX_MARGIN, SENSOR_MAX_RANGE
from .mpc_lib.mpc_core import ProgressiveMPC
from .mpc_lib.path_model import generate_eight_path, wrap_angle
from .mpc_lib.version_config import VERSION


def _yaw_from_quaternion(q):
    """geometry_msgs/Quaternion → 偏航角（平面车只需 yaw）。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class MpcNode(Node):
    def __init__(self):
        super().__init__("k7_mpc_node")
        self.reference = generate_eight_path(a=mpc_config.PATH_A)
        self.ref_theta0 = float(self.reference[2][0])
        self.controller = ProgressiveMPC(VERSION)

        self.odom = None       # 最新里程计位姿 (x, y, theta)
        self.origin = None     # 首帧位姿 (x0, y0, theta0)，用于路径对齐
        self.ranges = {}       # name -> 最近有效距离（米），无效读数为 None
        self.finished = False
        self.tick_count = 0
        self.solve_time_max = 0.0
        self.solve_time_sum = 0.0

        self.create_subscription(Odometry, mpc_config.ODOM_TOPIC, self._odom_cb, 10)
        for name, topic in mpc_config.IR_TOPICS.items():
            self.create_subscription(
                Range, topic,
                lambda msg, name=name: self._range_cb(msg, name), 10)
        self.cmd_pub = self.create_publisher(Twist, mpc_config.CMD_TOPIC, 10)
        self.create_timer(DT_CONTROL, self._control_loop)

        self.get_logger().info(
            f"MPC 节点已启动：版本 {VERSION['key']}（{VERSION['name']}），"
            f"路径幅度 A={mpc_config.PATH_A} m，控制周期 {DT_CONTROL * 1000:.0f} ms；"
            "等待首帧 /odom_combined 以对准路径起点……"
        )

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = _yaw_from_quaternion(msg.pose.pose.orientation)
        if self.origin is None:
            self.origin = (x, y, theta)
            self.get_logger().info(
                f"路径已对准：起点 ({x:.2f}, {y:.2f})，朝向 {math.degrees(theta):.1f}°"
            )
        self.odom = (x, y, theta)

    def _range_cb(self, msg, name):
        # 超量程/无效读数记为 None，控制时按“无检测”处理
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            self.ranges[name] = float(msg.range)
        else:
            self.ranges[name] = None

    def _to_path_frame(self, x, y, theta):
        """里程计位姿 → 路径局部系（SE(2) 刚体变换：首帧位姿 ↦ 路径起点+初始朝向）。"""
        x0, y0, theta0 = self.origin
        alpha = self.ref_theta0 - theta0
        dx, dy = x - x0, y - y0
        ca, sa = math.cos(alpha), math.sin(alpha)
        return (
            ca * dx - sa * dy,
            sa * dx + ca * dy,
            wrap_angle(theta - theta0 + self.ref_theta0),
        )

    def _publish(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _control_loop(self):
        if self.origin is None or self.odom is None:
            return  # 等首帧里程计
        if self.finished:
            return
        if self.controller.last_ref_idx >= len(self.reference[0]) - FINISH_INDEX_MARGIN:
            self._publish(0.0, 0.0)
            self.finished = True
            self.get_logger().info("已到达路径终点，停车。")
            return

        x, y, theta = self._to_path_frame(*self.odom)
        sensors = {
            name: (self.ranges.get(name) or SENSOR_MAX_RANGE)
            for name in mpc_config.IR_TOPICS
        }
        try:
            command = self.controller.control(x, y, theta, self.reference, sensors)
        except Exception as exc:  # 求解异常时本拍停车，下一拍重试
            self.get_logger().error(f"MPC 求解异常，本拍输出零速：{exc}")
            self._publish(0.0, 0.0)
            return

        self._publish(command["v"], command["omega"])
        self.solve_time_max = max(self.solve_time_max, command["optimizer_solve_time"])
        self.solve_time_sum += command["optimizer_solve_time"]
        self.tick_count += 1
        if self.tick_count % 70 == 0:  # 约 5 s 报一次求解耗时
            self.get_logger().info(
                f"SLSQP 求解耗时：均值 {self.solve_time_sum / self.tick_count * 1000:.1f} ms，"
                f"峰值 {self.solve_time_max * 1000:.1f} ms（预算 {DT_CONTROL * 1000:.0f} ms）"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
