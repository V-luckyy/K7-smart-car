"""k7_camera 双目拆分节点（骨架）。

功能：订阅 usb_cam 输出的 3840×1080 左右并排帧，裁成左右两张 1920×1080，
     分别发布 /camera/left|right/image_raw + camera_info。

TODO（感知组填写）：
1. 确认 usb_cam 实际发布的话题名，通过参数 input_topic 传入或 launch remap。
2. camera_info 从 config/left.yaml、right.yaml 读取（camera_info_manager 加载）。
3. frame_id 与 URDF 里的相机光学 frame 对齐（目前默认 camera_left/right_optical_frame）。
4. 若 usb_cam 用 sensor_data QoS，这里订阅/发布可能需要匹配 QoS（见 _img_cb 注释）。
"""
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
from ament_index_python.packages import get_package_share_directory


class StereoSplitter(Node):
    def __init__(self):
        super().__init__('stereo_splitter')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('left_frame_id', 'camera_left_optical_frame')
        self.declare_parameter('right_frame_id', 'camera_right_optical_frame')

        pkg_dir = get_package_share_directory('k7_camera')
        self.declare_parameter('left_info_url',
                               'file://' + os.path.join(pkg_dir, 'config', 'left.yaml'))
        self.declare_parameter('right_info_url',
                               'file://' + os.path.join(pkg_dir, 'config', 'right.yaml'))

        input_topic = self.get_parameter('input_topic').value
        self.left_frame_id = self.get_parameter('left_frame_id').value
        self.right_frame_id = self.get_parameter('right_frame_id').value

        self.bridge = CvBridge()

        # camera_info 加载器（首次 getCameraInfo() 时才真正读 yaml）
        self.left_cinfo = CameraInfoManager(
            self, 'left_camera', self.get_parameter('left_info_url').value)
        self.right_cinfo = CameraInfoManager(
            self, 'right_camera', self.get_parameter('right_info_url').value)

        # ---- 发布左右图 + camera_info ----
        self.left_img_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_img_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 10)

        # ---- 订阅并排帧 ----
        self.create_subscription(Image, input_topic, self._img_cb, 10)

        self.get_logger().info(f'stereo_splitter 已启动，订阅 {input_topic} ...')

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败：{exc}')
            return

        h, w = frame.shape[:2]
        half = w // 2
        left = frame[:, :half]
        right = frame[:, half:]

        # 左右图（复用输入 header，frame_id 换成各自光学 frame）
        left_msg = self.bridge.cv2_to_imgmsg(left, 'bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right, 'bgr8')
        left_msg.header = msg.header
        right_msg.header = msg.header
        left_msg.header.frame_id = self.left_frame_id
        right_msg.header.frame_id = self.right_frame_id
        self.left_img_pub.publish(left_msg)
        self.right_img_pub.publish(right_msg)

        # camera_info（时间戳对齐图像）
        try:
            left_info = self.left_cinfo.getCameraInfo()
            right_info = self.right_cinfo.getCameraInfo()
        except Exception as exc:
            self.get_logger().error(f'camera_info 加载失败：{exc}')
            return
        left_info.header = left_msg.header
        right_info.header = right_msg.header
        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)


def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
