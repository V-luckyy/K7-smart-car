"""调试工具：以 10Hz 发布 3 路“无障碍”红外 Range（读数=满量程）。

固件扩展上行帧之前，用于联调 mpc_node 控制链路：
    ros2 run k7_mpc fake_ir_publisher
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from . import mpc_config
from .mpc_lib.common_config import SENSOR_MAX_RANGE


class FakeIrPublisher(Node):
    def __init__(self):
        super().__init__("fake_ir_publisher")
        self.pubs = {
            name: self.create_publisher(Range, topic, 10)
            for name, topic in mpc_config.IR_TOPICS.items()
        }
        self.create_timer(0.1, self._tick)
        self.get_logger().info("假红外发布器已启动（全部满量程 = 无障碍）")

    def _tick(self):
        for pub in self.pubs.values():
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.1
            msg.min_range = 0.1
            msg.max_range = float(SENSOR_MAX_RANGE)
            msg.range = float(SENSOR_MAX_RANGE)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeIrPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
