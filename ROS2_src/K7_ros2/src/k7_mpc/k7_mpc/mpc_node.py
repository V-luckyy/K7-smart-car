"""K7 MPC 节点：/odom_combined + 3 路红外(/ir_distances) → ProgressiveMPC → /cmd_vel_mpc。

闭环逐环节对齐仿真工程 five_version_progressive_r04_with_pid：
- 状态 (x, y, θ) 来自 /odom_combined（EKF 融合里程计）；收到首帧里程计时，
  把硬编码 8 字路径经 SE(2) 刚体变换对齐到小车当前位姿（不动算法本体）。
- 3 路红外 /ir_distances(IrDistances) → {"front", "left45", "right45"} 距离字典（米）；
  无数据/超量程按 SENSOR_MAX_RANGE 处理（仿真“无检测”语义）。
- 输出 (v, omega) → geometry_msgs/Twist(linear.x=v, angular.z=omega)，
  正 omega = 左转，与仿真一致。
- CSV 日志：每控制周期记录轨迹状态（odom 系 + 路径系）、参考点索引、三路红外距离、
  控制量（v/omega 及 raw）、避障标志、风险量、权重、求解耗时等，便于离线分析避障效果。
  日志文件在 ~/mpc_log/mpc_YYYYMMDD_HHMMSS.csv（尽力而为，失败不影响控制）。
"""

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from k7_msgs.msg import IrDistances

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


LOG_HEADER = [
    "tick", "ros_time_ns",
    "odom_x", "odom_y", "odom_theta",
    "path_x", "path_y", "path_theta", "ref_idx",
    "front", "left45", "right45", "d_min",
    "v", "omega", "v_raw", "omega_raw",
    "avoidance_active", "right_bypass_active", "cbf_active",
    "risk", "closing_rate", "obs_warn_weight", "obs_safe_weight",
    "solve_time_ms", "solver_status",
]


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
        self.create_subscription(IrDistances, mpc_config.IR_TOPIC, self._ir_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, mpc_config.CMD_TOPIC, 10)
        self.create_timer(DT_CONTROL, self._control_loop)

        self._log_file = None
        self._log_writer = None
        self._init_log()

        self.get_logger().info(
            f"MPC 节点已启动：版本 {VERSION['key']}（{VERSION['name']}），"
            f"路径幅度 A={mpc_config.PATH_A} m，控制周期 {DT_CONTROL * 1000:.0f} ms；"
            "等待首帧 /odom_combined 以对准路径起点……"
        )

    def _init_log(self):
        """创建 CSV 日志（尽力而为，失败不影响控制）。"""
        try:
            log_dir = os.path.expanduser("~/mpc_log")
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, time.strftime("mpc_%Y%m%d_%H%M%S.csv"))
            self._log_file = open(log_path, "w", newline="")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(LOG_HEADER)
            self.get_logger().info(f"MPC 日志文件：{log_path}")
        except OSError as exc:
            self._log_file = None
            self._log_writer = None
            self.get_logger().warn(f"无法创建日志文件：{exc}")

    def _log_row(self, path_state, sensors, command):
        """写一行记录；不抛异常，避免影响控制循环。"""
        if self._log_writer is None:
            return
        x, y, theta = path_state
        try:
            self._log_writer.writerow([
                self.tick_count,
                self.get_clock().now().nanoseconds,
                self.odom[0], self.odom[1], self.odom[2],
                x, y, theta,
                self.controller.last_ref_idx,
                sensors["front"], sensors["left45"], sensors["right45"],
                command["d_min_sensor"],
                command["v"], command["omega"],
                command["v_raw"], command["omega_raw"],
                int(command["avoidance_active"]),
                int(command["right_bypass_active"]),
                int(command["cbf_active"]),
                command["risk"],
                command["closing_rate"],
                command["obs_warn_weight"],
                command["obs_safe_weight"],
                command["optimizer_solve_time"] * 1000.0,
                command["solver_status"],
            ])
            self._log_file.flush()
        except Exception:
            pass

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

    def _ir_cb(self, msg):
        # 非有限或 <=0 的读数记为 None，控制时按“无检测”处理
        for name in mpc_config.IR_NAMES:
            val = float(getattr(msg, name))
            if math.isfinite(val) and val > 0.0:
                self.ranges[name] = val
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
            for name in mpc_config.IR_NAMES
        }
        try:
            command = self.controller.control(x, y, theta, self.reference, sensors)
        except Exception as exc:  # 求解异常时本拍停车，下一拍重试
            self.get_logger().error(f"MPC 求解异常，本拍输出零速：{exc}")
            self._publish(0.0, 0.0)
            return

        self._publish(command["v"], command["omega"])
        self._log_row((x, y, theta), sensors, command)

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
        if node._log_file is not None:
            node._log_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
