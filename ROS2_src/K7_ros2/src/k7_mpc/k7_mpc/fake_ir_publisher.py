"""调试工具：以 10Hz 发布 3 路“无障碍”红外（读数=满量程）单话题 /ir_distances。

固件测距帧上行之前，用于联调 mpc_node 控制链路：
    ros2 run k7_mpc fake_ir_publisher
"""

import rclpy
from rclpy.node import Node
from k7_msgs.msg import IrDistances

from . import mpc_config
from .mpc_lib.common_config import SENSOR_MAX_RANGE


class FakeIrPublisher(Node):
    def __init__(self):
        super().__init__("fake_ir_publisher")
        self.pub = self.create_publisher(IrDistances, mpc_config.IR_TOPIC, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info("假红外发布器已启动（全部满量程 = 无障碍）")

    def _tick(self):
        msg = IrDistances()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.front = float(SENSOR_MAX_RANGE)
        msg.left45 = float(SENSOR_MAX_RANGE)
        msg.right45 = float(SENSOR_MAX_RANGE)
        self.pub.publish(msg)


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
