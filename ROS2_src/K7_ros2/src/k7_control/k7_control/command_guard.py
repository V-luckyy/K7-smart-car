import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from k7_msgs.msg import MotionCommand
from k7_msgs.srv import SelectSource
from .core import Authority, limit_drive
from .ros_utils import diagnostic, parameters, stamp_fresh


class CommandGuard(Node):
    """Sole cmd_vel publisher: motion safety + single command source.

    Keeps only the essentials: no reverse, speed/angular caps, a single
    active command source, and a stale-candidate timeout. Chassis-disable,
    perception, standstill and heartbeat gates are intentionally removed.
    """
    def __init__(self):
        super().__init__('command_guard')
        self.p = parameters(self, dict(rate=30.0, candidate_timeout=0.25,
            track=0.329, wheel_limit=0.35, max_angular=1.2,
            base_frame='base_link', no_perception_test=False, test_speed_limit=0.05))
        if self.p['no_perception_test'] and not (
                0 < self.p['test_speed_limit'] <= self.p['wheel_limit']):
            raise ValueError('test_speed_limit must be positive and within wheel_limit')
        self.authority = Authority(self.p['candidate_timeout'])
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.status = self.create_publisher(DiagnosticArray, 'control/status', 1)
        self.create_subscription(MotionCommand, 'control/candidate', self.candidate_cb, 1)
        self.create_service(SelectSource, 'control/select_source', self.select_cb)
        self.create_service(SetBool, 'control/hold', self.hold_cb)
        self.create_timer(1/self.p['rate'], self.tick)
        self.create_timer(1.0, self.publish_status)

    def select_cb(self, request, response):
        response.success = self.authority.select(request.source, True, True)
        response.permission_id.uuid = list(self.authority.token)
        response.reason = self.authority.reason if response.success else 'busy or held'
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
        # Forward only; the STM32 enforces its own hard limits.
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
        self.send(*self.authority.output(time.monotonic(), True, True))

    def publish_status(self):
        a = self.authority
        self.status.publish(diagnostic(self, 'command_guard', dict(source=a.source,
            permission=a.token.hex(), held=a.held,
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
