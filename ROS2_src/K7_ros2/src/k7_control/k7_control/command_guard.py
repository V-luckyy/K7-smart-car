import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from k7_msgs.msg import MotionCommand
from k7_msgs.srv import SelectSource
from .core import Authority, control_ready, finite, limit_drive, valid_scan
from .ros_utils import diagnostic, fields, parameters, stamp_fresh


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        self.p = parameters(self, dict(rate=30.0, candidate_timeout=0.25,
            sensor_timeout=0.4, task_timeout=2.5, stop_distance=0.3,
            track=0.329, wheel_limit=0.35, max_angular=1.2,
            standstill_speed=0.02, standstill_time=0.3, base_frame='base_link', odom_frame='odom',
            no_perception_test=False, test_speed_limit=0.05))
        if self.p['no_perception_test'] and not (
                finite(self.p['test_speed_limit']) and 0 < self.p['test_speed_limit'] <= self.p['wheel_limit']):
            raise ValueError('test_speed_limit must be positive and within wheel_limit')
        self.authority = Authority(self.p['candidate_timeout'])
        self.odom_time = self.scan_time = self.task_time = -math.inf
        self.still_since = None
        self.scan_ok = False
        self.disabled = True
        self.hardware_time = -math.inf
        self.task_token = ''
        self.granted_at = 0.0
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.status = self.create_publisher(DiagnosticArray, 'control/status', 1)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        self.create_subscription(Bool, 'chassis/disabled', self.hardware_cb, 1)
        if self.p['no_perception_test']:
            self.get_logger().warning('NO-PERCEPTION TEST: obstacle stop is unavailable; test in an isolated area')
        else:
            self.create_subscription(LaserScan, 'perception/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, 'task/status', self.task_cb, 1)
        self.create_subscription(MotionCommand, 'control/candidate', self.candidate_cb, 1)
        self.create_service(SelectSource, 'control/select_source', self.select_cb)
        self.create_service(SetBool, 'control/hold', self.hold_cb)
        self.create_timer(1/self.p['rate'], self.tick)
        self.create_timer(1.0, self.publish_status)
        self.last_state = None

    def odom_cb(self, msg):
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if (msg.header.frame_id != self.p['odom_frame'] or
                not stamp_fresh(self, msg.header.stamp, self.p['sensor_timeout']) or not finite(v, w)):
            self.still_since = None
            return
        now = time.monotonic()
        if now-self.odom_time > self.p['sensor_timeout']:
            self.still_since = None
        self.odom_time = now
        if abs(v) < self.p['standstill_speed'] and abs(w) < self.p['standstill_speed']:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def hardware_cb(self, msg):
        self.disabled, self.hardware_time = msg.data, time.monotonic()

    def scan_cb(self, msg):
        self.scan_time = time.monotonic()
        self.scan_ok = (stamp_fresh(self, msg.header.stamp, self.p['sensor_timeout'])
            and msg.header.frame_id == self.p['base_frame']
            and valid_scan(msg.ranges, msg.range_min, msg.range_max)
            and min(msg.ranges, default=0.0) > self.p['stop_distance'])

    def task_cb(self, msg):
        if stamp_fresh(self, msg.header.stamp, self.p['task_timeout']):
            self.task_token = fields(msg).get('permission', '')
            self.task_time = time.monotonic()

    def ready(self, now):
        return control_ready(now-self.odom_time <= self.p['sensor_timeout'],
            now-self.hardware_time <= self.p['sensor_timeout'], not self.disabled,
            now-self.scan_time <= self.p['sensor_timeout'], self.scan_ok,
            self.p['no_perception_test'])

    def stopped(self, now):
        return (now-self.odom_time <= self.p['sensor_timeout'] and
                self.still_since is not None and
                now-self.still_since >= self.p['standstill_time'])

    def select_cb(self, request, response):
        now = time.monotonic()
        response.success = self.authority.select(request.source, self.stopped(now), self.ready(now))
        response.permission_id.uuid = list(self.authority.token)
        response.reason = self.authority.reason if response.success else 'busy, held, moving or sensors not ready'
        if response.success and request.source:
            self.granted_at = now
        if response.success and not request.source:
            self.send(0.0, 0.0)
        self.publish_status()
        return response

    def hold_cb(self, request, response):
        self.authority.hold(request.data)
        self.send(0.0, 0.0)
        self.publish_status()
        response.success, response.message = True, self.authority.reason
        return response

    def candidate_cb(self, msg):
        if (msg.header.frame_id == self.p['base_frame'] and
                stamp_fresh(self, msg.header.stamp, self.p['candidate_timeout'])):
            self.authority.receive(bytes(msg.permission_id.uuid), msg.twist.linear.x,
                                   msg.twist.angular.z, time.monotonic())

    def send(self, v, w):
        # The camera covers the front only: reverse commands are not authorized.
        if v < 0:
            v, w = 0.0, 0.0
        if self.p['no_perception_test']:
            v = min(v, self.p['test_speed_limit'])
        w = max(-self.p['max_angular'], min(self.p['max_angular'], w))
        v, w = limit_drive(v, w, self.p['track'], self.p['wheel_limit'], True)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.authority.source and self.authority.candidate is None and now-self.granted_at > 1.0:
            self.authority.revoke('first candidate timeout')
        heartbeat = (now-self.task_time <= self.p['task_timeout'] and
                     self.task_token == self.authority.token.hex())
        # Allow the initial status message to catch up with the grant response.
        heartbeat = heartbeat or now-self.granted_at < 0.3
        self.send(*self.authority.output(now, self.ready(now), heartbeat))
        state = (self.authority.source, self.authority.token, self.authority.held, self.authority.reason)
        if state != self.last_state:
            self.last_state = state
            self.publish_status()

    def publish_status(self):
        a = self.authority
        self.status.publish(diagnostic(self, 'command_guard', dict(source=a.source,
            permission=a.token.hex(), held=a.held, stopped=self.stopped(time.monotonic()),
            test_mode='no_perception' if self.p['no_perception_test'] else 'none'),
            int(a.held), a.reason))


def main(args=None):
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()
