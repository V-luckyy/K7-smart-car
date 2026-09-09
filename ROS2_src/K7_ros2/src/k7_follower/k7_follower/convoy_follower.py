#!/usr/bin/env python3
# coding=utf-8
"""K7 多机编队 —— 跟随者节点（Leader-Follower v1，速度复刻 + 纵向间距保持）。

为什么不用"位置闭环"：
  leader 与 follower 各自的 odom 原点（开机点）不同、朝向不同，坐标系不重合，
  follower 直接拿 leader 的绝对坐标做跟踪在数学上无意义（除非先统一坐标系）。
  v1 采用最简单的"速度复刻"思路，让队形在运动中被拉开/挤压时能自动回稳到目标间距：
    - 角速度：完全复刻 leader（转向跟着转，保持姿态一致）；
    - 线速度：在 leader 线速度上叠加"间距误差"比例项，远了加速、近了减速。
  间距 d 由 ḋ = v_leader − v_follower 积分得到（沿车头方向的一维近似），转弯时会
  有少量误差，属 v1 已知局限；后续可升级为"共同坐标系 / 相对位姿"方案。

控制链：/leader/odom_combined ─┐
        /odom_combined(自身)  ─┼→ follower_node → cmd_vel_follower → twist_mux → cmd_vel → STM32
        /ir_distances(自身)   ─┘
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from k7_msgs.msg import IrDistances
from nav_msgs.msg import Odometry


class ConvoyFollower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # ---- 参数（可被 launch / ros2 param set / config yaml 覆盖）----
        self.declare_parameter('leader_topic', '/leader/odom_combined')  # 绝对话题：跨命名空间订阅 leader
        self.declare_parameter('own_odom_topic', 'odom_combined')        # 相对话题：自动落到本机 /<ns>/odom_combined
        self.declare_parameter('ir_topic', 'ir_distances')               # 本机三路红外（前/左45/右45，单位 m）
        self.declare_parameter('cmd_topic', 'cmd_vel_follower')          # 发给 twist_mux 的仲裁输入话题
        self.declare_parameter('rate', 20.0)                             # 控制频率 Hz

        # 跟随策略
        self.declare_parameter('target_gap', 1.2)     # 目标间距 (m)：期望车头到前车车尾保持多远
        self.declare_parameter('kp_gap', 0.8)         # 间距误差比例增益：越大回位越快，过大易振荡
        self.declare_parameter('v_max', 0.5)          # 最大线速度 (m/s)
        self.declare_parameter('w_max', 1.2)          # 最大角速度 (rad/s)
        self.declare_parameter('leader_timeout', 1.0) # leader 断链秒数，超时急停（安全）
        self.declare_parameter('use_ir', True)        # 是否启用前红外安全刹停
        self.declare_parameter('ir_safe', 0.35)       # 前方 < 此距离(m)：立即停（包括不追尾前车）
        self.declare_parameter('ir_slow', 0.6)        # 前方 < 此距离(m)：限速慢行

        self._leader_topic = self.get_parameter('leader_topic').value
        self._own_odom_topic = self.get_parameter('own_odom_topic').value
        self._ir_topic = self.get_parameter('ir_topic').value
        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._rate = float(self.get_parameter('rate').value)
        self._gap_t = float(self.get_parameter('target_gap').value)
        self._kp = float(self.get_parameter('kp_gap').value)
        self._v_max = float(self.get_parameter('v_max').value)
        self._w_max = float(self.get_parameter('w_max').value)
        self._lead_timeout = float(self.get_parameter('leader_timeout').value)
        self._use_ir = bool(self.get_parameter('use_ir').value)
        self._ir_safe = float(self.get_parameter('ir_safe').value)
        self._ir_slow = float(self.get_parameter('ir_slow').value)

        # ---- 运行状态 ----
        self._gap = self._gap_t          # 当前估计间距 (m)，初始假定已停在目标间距
        self._have_lead = False
        self._lead_last = None           # leader 最近一次消息时刻
        self._v_l = 0.0                  # leader 线速度
        self._w_l = 0.0                  # leader 角速度
        self._v_f = 0.0                  # 自身线速度（来自本机 odom）
        self._own_odom_ok = False
        self._front = float('inf')       # 前方红外距离
        self._warned_no_odom = False
        self._log_t = 0.0

        # QoS：控制类用可靠传输；间距 2 帧足够
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Odometry, self._leader_topic, self._on_leader, qos)
        self.create_subscription(Odometry, self._own_odom_topic, self._on_own_odom, qos)
        self.create_subscription(IrDistances, self._ir_topic, self._on_ir, qos)

        self._pub = self.create_publisher(Twist, self._cmd_topic, 2)
        self.create_timer(1.0 / self._rate, self._control)

        self.get_logger().info(
            f'编队跟随节点就绪: 跟随 <{self._leader_topic}> → 发 <{self._cmd_topic}>, '
            f'目标间距 {self._gap_t:.2f} m, 控率 {self._rate:.0f} Hz')

    # ---------------- 订阅回调 ----------------
    def _on_leader(self, msg: Odometry):
        self._v_l = msg.twist.twist.linear.x
        self._w_l = msg.twist.twist.angular.z
        if not self._have_lead:
            self._have_lead = True
            # 收到第一帧 leader：假定当时已按目标间距排好，间距从目标值开始积分
            self._gap = self._gap_t
            self.get_logger().info('已收到 leader 消息，开始跟随')
        self._lead_last = self.get_clock().now()

    def _on_own_odom(self, msg: Odometry):
        self._v_f = msg.twist.twist.linear.x
        self._own_odom_ok = True

    def _on_ir(self, msg: IrDistances):
        self._front = msg.front

    # ---------------- 控制主循环 ----------------
    def _control(self):
        # 1. leader 断链保护：超过 leader_timeout 未收到 → 急停
        if not self._have_lead or self._lead_last is None:
            self._stop('等待 leader 数据…')
            return
        if (self.get_clock().now() - self._lead_last).nanoseconds * 1e-9 > self._lead_timeout:
            self._stop('leader 超时(断链)，急停')
            return

        # 2. 间距积分：ḋ = v_leader − v_follower（一维近似，沿各自车头方向）
        dt = 1.0 / self._rate
        self._gap += (self._v_l - self._v_f) * dt
        self._gap = max(0.05, min(self._gap, 10.0))          # 钳制防发散

        # 3. 基础控制
        v_cmd = self._v_l + self._kp * (self._gap - self._gap_t)   # 远了加速 / 近了减速
        w_cmd = self._w_l                                          # 角速度复刻

        # 4. 限幅（线速度只允许前向跟随 leader；leader 倒车时允许跟随小倒车）
        v_lo = min(0.0, self._v_l)
        v_cmd = max(v_lo, min(v_cmd, self._v_max))
        w_cmd = max(-self._w_max, min(w_cmd, self._w_max))

        # 5. 前红外安全：太近直接停（防追尾 / 防撞物）
        if self._use_ir and self._front < self._ir_safe:
            v_cmd, w_cmd = 0.0, 0.0
        elif self._use_ir and self._front < self._ir_slow:
            v_cmd = min(v_cmd, self._v_max * 0.3)

        self._publish(v_cmd, w_cmd)
        self._log(v_cmd, w_cmd)

    def _publish(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self._pub.publish(twist)

    def _stop(self, why):
        if self._have_lead:
            self.get_logger().warn(why)
            self._have_lead = False   # 只告警一次；收到新 leader 帧自动恢复
        self._publish(0.0, 0.0)

    def _log(self, v, w):
        if not self._own_odom_ok and not self._warned_no_odom:
            self.get_logger().warn(
                f'未收到本机 odom <{self._own_odom_topic}>，v_f 视为 0，间距积分会不准；请确认 k7_core 已启动')
            self._warned_no_odom = True
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._log_t >= 2.0:   # 2 秒打印一次运行状态，方便观察
            self._log_t = now
            self.get_logger().info(
                f'lead(v={self._v_l:+.2f},w={self._w_l:+.2f}) | 自身v={self._v_f:+.2f} | '
                f'间距≈{self._gap:.2f}/{self._gap_t:.2f}m | out(v={v:+.2f},w={w:+.2f}) | '
                f'前红外={self._front if self._front < 5 else ">5"}m')


def main(args=None):
    rclpy.init(args=args)
    node = ConvoyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
