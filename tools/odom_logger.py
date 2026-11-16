import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Logger(Node):
    def __init__(self, name):
        super().__init__('odom_logger')
        self.f = open('/tmp/odom_%s.csv' % name, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw'])
        self.create_subscription(Odometry, '/%s/odom' % name, self.cb, 10)
        self.t0 = None

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.w.writerow([round(t - self.t0, 4), round(p.position.x, 4),
                         round(p.position.y, 4), round(yaw, 4)])
        self.f.flush()


def main():
    rclpy.init()
    node = Logger(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.f.close()
    node.destroy_node()
    rclpy.try_shutdown()


main()
