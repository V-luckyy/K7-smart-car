import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from k7_msgs.msg import ControlReference, MotionCommand
from .core import apf_stanley, finite, project, stanley_only, valid_scan
from .ros_utils import diagnostic, fields, parameters, permission, stamp_fresh, yaw


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.p = parameters(self, dict(algorithm='apf_stanley', rate=20.0,
            sensor_timeout=0.4, control_point=0.25, stanley_gain=1.0,
            soft_speed=0.1, max_steer=0.8, track=0.329, wheel_limit=0.35,
            stop_distance=0.3, slow_distance=0.9, influence_distance=1.2,
            max_repulsion=8.0, apf_gain=0.4, bypass_side=1.0,
            debug=False, debug_fields=['cte', 'heading_error', 'solve_ms'],
            base_frame='base_link', odom_frame='odom', max_path_points=512,
            reference_timeout=0.5, max_solve_ms=80.0, no_perception_test=False,
            test_speed_limit=0.05))
        if self.p['algorithm'] != 'apf_stanley':
            raise ValueError('Only apf_stanley is implemented for stereo; legacy MPC is not adapted')
        if not (0 < self.p['stop_distance'] < self.p['slow_distance'] <= self.p['influence_distance']
                and self.p['control_point'] > 0 and self.p['soft_speed'] > 0):
            raise ValueError('invalid controller geometry or distance limits')
        if self.p['no_perception_test'] and not (
                finite(self.p['test_speed_limit']) and 0 < self.p['test_speed_limit'] <= self.p['wheel_limit']):
            raise ValueError('test_speed_limit must be positive and within wheel_limit')
        self.path, self.ref, self.pose, self.scan = [], None, None, None
        self.token = bytes(16)
        self.auto = False
        self.odom_time = self.scan_time = -math.inf
        self.guard_time = -math.inf
        self.last_debug = 0.0
        self.pub = self.create_publisher(MotionCommand, 'control/candidate', 1)
        self.debug_pub = (self.create_publisher(DiagnosticArray, 'control/debug', 1)
                          if self.p['debug'] else None)
        self.create_subscription(ControlReference, 'control/reference', self.reference_cb, 1)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        if self.p['no_perception_test']:
            self.get_logger().warning('NO-PERCEPTION TEST: APF disabled; use a clear, isolated test area')
        else:
            self.create_subscription(LaserScan, 'perception/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, 'control/status', self.guard_cb, 1)
        self.create_timer(1/self.p['rate'], self.tick)

    def guard_cb(self, msg):
        if not stamp_fresh(self, msg.header.stamp, 2.5):
            return
        data = fields(msg)
        token = permission(data.get('permission', ''))
        if token != self.token:
            self.path, self.ref = [], None
        self.token, self.auto = token, data.get('source') == '2'
        self.guard_time = time.monotonic()

    def reference_cb(self, msg):
        if (not self.auto or bytes(msg.permission_id.uuid) != self.token or
                not stamp_fresh(self, msg.header.stamp, self.p['reference_timeout']) or
                msg.path.header.frame_id != self.p['odom_frame'] or
                not 2 <= len(msg.path.poses) <= self.p['max_path_points'] or
                not finite(msg.speed_limit) or not 0 <= msg.speed_limit <= self.p['wheel_limit']):
            return
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.path.poses]
        if not all(finite(*p) for p in path):
            return
        try:
            project(path, *path[0])
        except ValueError:
            return
        self.path, self.ref = path, msg

    def odom_cb(self, msg):
        q = msg.pose.pose
        pose = (q.position.x, q.position.y, yaw(q.orientation))
        if (msg.header.frame_id == self.p['odom_frame'] and finite(*pose, msg.twist.twist.linear.x)
                and stamp_fresh(self, msg.header.stamp, self.p['sensor_timeout'])):
            self.pose, self.measured_speed = pose, msg.twist.twist.linear.x
            self.odom_time = time.monotonic()

    def scan_cb(self, msg):
        if (msg.header.frame_id == self.p['base_frame'] and
                stamp_fresh(self, msg.header.stamp, self.p['sensor_timeout']) and
                valid_scan(msg.ranges, msg.range_min, msg.range_max)):
            self.scan, self.scan_time = msg, time.monotonic()

    def tick(self):
        now = time.monotonic()
        if (not self.auto or self.ref is None or now-self.guard_time > 2.5 or
                now-self.odom_time > self.p['sensor_timeout'] or
                (not self.p['no_perception_test'] and
                 now-self.scan_time > self.p['sensor_timeout'])):
            return
        speed = min(self.ref.speed_limit, self.p['test_speed_limit']) if self.p['no_perception_test'] else self.ref.speed_limit
        start = time.monotonic()
        try:
            if not stamp_fresh(self, self.ref.header.stamp, self.p['reference_timeout']):
                return
            if self.p['no_perception_test']:
                v, w, debug = stanley_only(self.path, self.pose, speed, self.measured_speed, self.p)
            else:
                angles = [self.scan.angle_min+i*self.scan.angle_increment for i in range(len(self.scan.ranges))]
                v, w, debug = apf_stanley(self.path, self.pose, speed, self.measured_speed,
                                         angles, self.scan.ranges, self.p)
            debug['solve_ms'] = (time.monotonic()-start)*1000
            if debug['solve_ms'] > self.p['max_solve_ms']:
                raise ValueError('controller exceeded solve budget')
        except (ValueError, RuntimeError, KeyError) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            return
        msg = MotionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['base_frame']
        msg.permission_id.uuid = list(self.token)
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.pub.publish(msg)
        if self.debug_pub and now-self.last_debug >= 0.2:
            debug.update(v=v, w=w)
            self.debug_pub.publish(diagnostic(self, 'controller',
                {k: debug[k] for k in self.p['debug_fields'] if k in debug}))
            self.last_debug = now


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
