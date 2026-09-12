#!/usr/bin/env python3
# coding=utf-8
"""Lightweight predecessor-trail following with local three-sensor APF."""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import Twist
from k7_msgs.msg import IrDistances
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def clamp(value, low, high):
    return max(low, min(value, high))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ConvoyFollower(Node):
    """Follow the predecessor's odometry trail and avoid local obstacles."""

    def __init__(self):
        super().__init__('follower_node')

        self.declare_parameter('leader_topic', '/leader/odom')
        self.declare_parameter('own_odom_topic', 'odom')
        self.declare_parameter('ir_topic', 'ir_distances')
        self.declare_parameter('cmd_topic', 'cmd_vel_follower')
        self.declare_parameter('rate', 20.0)

        self.declare_parameter('target_gap', 1.2)
        self.declare_parameter('path_spacing', 0.03)
        self.declare_parameter('lookahead', 0.30)
        self.declare_parameter('kp_gap', 0.8)
        self.declare_parameter('k_heading', 1.5)
        self.declare_parameter('k_lateral', 1.0)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('track_width', 0.329)
        self.declare_parameter('leader_timeout', 0.5)
        self.declare_parameter('sensor_timeout', 0.5)

        self.declare_parameter('use_apf', True)
        self.declare_parameter('apf_influence', 0.7)
        self.declare_parameter('apf_stop', 0.18)
        self.declare_parameter('apf_turn_gain', 0.8)
        self.declare_parameter('apf_side_deadband', 0.12)

        self._leader_topic = self.get_parameter('leader_topic').value
        self._own_topic = self.get_parameter('own_odom_topic').value
        self._ir_topic = self.get_parameter('ir_topic').value
        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._rate = max(1.0, float(self.get_parameter('rate').value))

        self._gap_target = max(0.1, float(self.get_parameter('target_gap').value))
        self._path_spacing = max(0.01, float(self.get_parameter('path_spacing').value))
        self._lookahead = max(self._path_spacing, float(self.get_parameter('lookahead').value))
        self._kp_gap = float(self.get_parameter('kp_gap').value)
        self._k_heading = float(self.get_parameter('k_heading').value)
        self._k_lateral = float(self.get_parameter('k_lateral').value)
        self._v_max = max(0.01, float(self.get_parameter('v_max').value))
        self._w_max = max(0.01, float(self.get_parameter('w_max').value))
        self._track_width = max(0.01, float(self.get_parameter('track_width').value))
        self._leader_timeout = float(self.get_parameter('leader_timeout').value)
        self._sensor_timeout = float(self.get_parameter('sensor_timeout').value)

        self._use_apf = bool(self.get_parameter('use_apf').value)
        self._apf_influence = float(self.get_parameter('apf_influence').value)
        self._apf_stop = float(self.get_parameter('apf_stop').value)
        self._apf_gain = float(self.get_parameter('apf_turn_gain').value)
        self._apf_deadband = float(self.get_parameter('apf_side_deadband').value)

        # About four meters beyond the requested gap is sufficient history for recovery.
        path_capacity = max(64, int((self._gap_target + 4.0) / self._path_spacing) + 4)
        self._path = deque(maxlen=path_capacity)

        self._leader_origin = None
        self._own_origin = None
        self._leader_raw = None
        self._own_raw = None
        self._leader_pose = None
        self._own_pose = None
        self._leader_s = 0.0
        self._leader_v = 0.0
        self._leader_w = 0.0
        self._leader_last = None
        self._own_last = None

        self._distances = (float('inf'), float('inf'), float('inf'))
        self._ir_last = None
        self._last_log = 0.0
        self._last_stop_reason = None

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self._leader_topic, self._on_leader, sensor_qos)
        self.create_subscription(Odometry, self._own_topic, self._on_own_odom, sensor_qos)
        self.create_subscription(IrDistances, self._ir_topic, self._on_ir, sensor_qos)
        self._publisher = self.create_publisher(Twist, self._cmd_topic, 1)
        self.create_timer(1.0 / self._rate, self._control)

        self.get_logger().info(
            f'跟随节点就绪: 前车={self._leader_topic}, 间距={self._gap_target:.2f} m, '
            f'本地APF={self._use_apf}')

    @staticmethod
    def _raw_pose(msg):
        pose = msg.pose.pose
        return (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    @staticmethod
    def _relative_pose(raw, origin, x_offset=0.0):
        dx = raw[0] - origin[0]
        dy = raw[1] - origin[1]
        c = math.cos(origin[2])
        s = math.sin(origin[2])
        return (
            x_offset + c * dx + s * dy,
            -s * dx + c * dy,
            wrap_angle(raw[2] - origin[2]),
        )

    def _on_leader(self, msg):
        self._leader_raw = self._raw_pose(msg)
        if self._leader_origin is None:
            self._leader_origin = self._leader_raw
        self._leader_v = float(msg.twist.twist.linear.x)
        self._leader_w = float(msg.twist.twist.angular.z)
        self._leader_last = self.get_clock().now()
        self._initialize_path_if_ready()
        if self._path:
            self._leader_pose = self._relative_pose(
                self._leader_raw, self._leader_origin, self._gap_target)
            self._append_leader_point()

    def _on_own_odom(self, msg):
        self._own_raw = self._raw_pose(msg)
        if self._own_origin is None:
            self._own_origin = self._own_raw
        self._own_last = self.get_clock().now()
        self._initialize_path_if_ready()
        if self._path:
            self._own_pose = self._relative_pose(self._own_raw, self._own_origin)

    def _on_ir(self, msg):
        def distance(value):
            value = float(value)
            return value if math.isfinite(value) and value >= 0.0 else 0.0

        self._distances = (
            distance(msg.front),
            distance(msg.left45),
            distance(msg.right45),
        )
        self._ir_last = self.get_clock().now()

    def _initialize_path_if_ready(self):
        if self._path or self._leader_origin is None or self._own_origin is None:
            return

        # Both odometers start in their own frame. The cars must be placed parallel,
        # with the predecessor target_gap meters ahead, before these first samples.
        steps = max(1, math.ceil(self._gap_target / self._path_spacing))
        for index in range(steps + 1):
            s = self._gap_target * index / steps
            self._path.append((s, s, 0.0, 0.0, 0.0))
        self._leader_s = self._gap_target
        self._leader_pose = (self._gap_target, 0.0, 0.0)
        self._own_pose = (0.0, 0.0, 0.0)
        self.get_logger().info('前车与本车里程计已就绪，开始记录前车轨迹')

    def _append_leader_point(self):
        last = self._path[-1]
        ds = math.hypot(self._leader_pose[0] - last[1], self._leader_pose[1] - last[2])
        if ds < self._path_spacing:
            return

        curvature = 0.0
        if abs(self._leader_v) > 0.05:
            curvature = clamp(self._leader_w / self._leader_v, -4.0, 4.0)
        self._leader_s = last[0] + ds
        self._path.append((
            self._leader_s,
            self._leader_pose[0],
            self._leader_pose[1],
            self._leader_pose[2],
            curvature,
        ))

    @staticmethod
    def _sample_path(points, target_s):
        if target_s <= points[0][0]:
            return points[0]
        for first, second in zip(points, points[1:]):
            if target_s <= second[0]:
                span = second[0] - first[0]
                ratio = 0.0 if span <= 0.0 else (target_s - first[0]) / span
                yaw_delta = wrap_angle(second[3] - first[3])
                return (
                    target_s,
                    first[1] + ratio * (second[1] - first[1]),
                    first[2] + ratio * (second[2] - first[2]),
                    wrap_angle(first[3] + ratio * yaw_delta),
                    first[4] + ratio * (second[4] - first[4]),
                )
        return points[-1]

    def _tracking_command(self):
        points = list(self._path)
        own_x, own_y, own_yaw = self._own_pose
        nearest = min(points, key=lambda p: (own_x - p[1]) ** 2 + (own_y - p[2]) ** 2)
        target = self._sample_path(
            points, min(nearest[0] + self._lookahead, self._leader_s))

        gap = max(0.0, self._leader_s - nearest[0])
        v_cmd = self._leader_v + self._kp_gap * (gap - self._gap_target)
        v_cmd = clamp(v_cmd, 0.0, self._v_max)

        dx = target[1] - own_x
        dy = target[2] - own_y
        along = math.cos(target[3]) * dx + math.sin(target[3]) * dy
        lateral = -math.sin(target[3]) * dx + math.cos(target[3]) * dy
        heading_error = wrap_angle(target[3] - own_yaw)
        lateral_angle = math.atan2(lateral, max(0.20, abs(along)))
        w_cmd = (
            v_cmd * target[4]
            + self._k_heading * heading_error
            + self._k_lateral * lateral_angle
        )
        return v_cmd, w_cmd, gap

    def _apf_command(self):
        front, left, right = self._distances
        span = max(0.01, self._apf_influence - self._apf_stop)

        def strength(distance):
            return clamp((self._apf_influence - distance) / span, 0.0, 1.0)

        front_force = strength(front)
        left_force = strength(left)
        right_force = strength(right)
        w_apf = self._apf_gain * (right_force * right_force - left_force * left_force)

        if front_force > 0.0:
            room = left - right
            if room > self._apf_deadband:
                direction = 1.0
            elif room < -self._apf_deadband:
                direction = -1.0
            else:
                direction = 1.0
            w_apf += direction * self._apf_gain * front_force

        nearest = min(front, left, right)
        if nearest <= self._apf_stop:
            speed_scale = 0.0
        elif nearest >= self._apf_influence:
            speed_scale = 1.0
        else:
            ratio = (nearest - self._apf_stop) / span
            speed_scale = 0.25 + 0.75 * ratio
        return speed_scale, w_apf

    def _limit_diff_drive(self, v_cmd, w_cmd):
        v_cmd = clamp(v_cmd, 0.0, self._v_max)
        if v_cmd <= 0.0:
            return 0.0, 0.0

        omega_limit = min(self._w_max, 2.0 * v_cmd / self._track_width)
        w_cmd = clamp(w_cmd, -omega_limit, omega_limit)
        outer_speed = v_cmd + abs(w_cmd) * self._track_width * 0.5
        if outer_speed > self._v_max:
            scale = self._v_max / outer_speed
            v_cmd *= scale
            w_cmd *= scale
        return v_cmd, w_cmd

    def _control(self):
        now = self.get_clock().now()
        if not self._path or self._own_pose is None:
            self._stop('等待前车和本车里程计')
            return
        if self._leader_last is None or self._age(now, self._leader_last) > self._leader_timeout:
            self._stop('前车状态超时')
            return
        if self._own_last is None or self._age(now, self._own_last) > self._sensor_timeout:
            self._stop('本车里程计超时')
            return
        if self._use_apf and (
                self._ir_last is None or self._age(now, self._ir_last) > self._sensor_timeout):
            self._stop('本车测距超时')
            return

        v_cmd, w_track, gap = self._tracking_command()
        speed_scale, w_apf = self._apf_command() if self._use_apf else (1.0, 0.0)
        v_cmd *= speed_scale
        v_cmd, w_cmd = self._limit_diff_drive(v_cmd, w_track + w_apf)
        self._publish(v_cmd, w_cmd)
        self._last_stop_reason = None
        self._log_status(now, gap, v_cmd, w_cmd, w_track, w_apf)

    @staticmethod
    def _age(now, timestamp):
        return (now - timestamp).nanoseconds * 1e-9

    def _publish(self, v_cmd, w_cmd):
        command = Twist()
        command.linear.x = v_cmd
        command.angular.z = w_cmd
        self._publisher.publish(command)

    def _stop(self, reason):
        self._publish(0.0, 0.0)
        if reason != self._last_stop_reason:
            self.get_logger().warn(reason)
            self._last_stop_reason = reason

    def _log_status(self, now, gap, v_cmd, w_cmd, w_track, w_apf):
        seconds = now.nanoseconds * 1e-9
        if seconds - self._last_log < 2.0:
            return
        self._last_log = seconds
        front, left, right = self._distances
        self.get_logger().info(
            f'gap={gap:.2f}/{self._gap_target:.2f} m, '
            f'lead_v={self._leader_v:.2f}, out=({v_cmd:.2f},{w_cmd:.2f}), '
            f'w(track/apf)=({w_track:.2f}/{w_apf:.2f}), '
            f'ir=({front:.2f},{left:.2f},{right:.2f})')


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
