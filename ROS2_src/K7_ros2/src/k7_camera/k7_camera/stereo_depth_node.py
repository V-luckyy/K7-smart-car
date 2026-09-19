#!/usr/bin/env python3
# coding=utf-8
"""双目深度 + 视差 + 点云节点。

订阅 stereo_splitter 发布的左右图（/camera/left|right/image_raw, rgb8），
做立体校正 → SGBM 立体匹配 → WLS 滤波 → 视差/深度/三维点云，并发布：
    /camera/depth/image_raw     —— 深度图（32FC1，米）
    /camera/disparity/image     —— 视差图（32FC1，像素）
    /camera/points2             —— 点云（PointCloud2，xyz + rgb，左相机光学 frame）

标定参数从 config/left.yaml、right.yaml 读取（projection_matrix 直接给出 P1/P2，
Q 矩阵由 P1/P2 推导，无需 stereo 外参 R/T）。
性能：1920×1080 的 SGBM 在 RK3576 上偏慢，默认 scale=0.5 降采样到 960×540。

依赖：cv_bridge、message_filters、sensor_msgs_py（点云构造）、opencv-contrib(ximgproc)。
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


def load_projection(path):
    """读 yaml 的 projection_matrix + 畸变/校正矩阵，返回 P, D, R, size。"""
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], np.float64)
    R = np.array(d['rectification_matrix']['data'], np.float64).reshape(3, 3)
    P = np.array(d['projection_matrix']['data'], np.float64).reshape(3, 4)
    size = (int(d['image_width']), int(d['image_height']))
    return K, D, R, P, size


def build_q(P1, P2):
    """由左右投影矩阵推导 Q（标准 stereoRectify 输出格式）。"""
    f = P1[0, 0]
    cx, cy = P1[0, 2], P1[1, 2]
    cxr = P2[0, 2]
    tx = -P2[0, 3] / f  # 右相机相对左相机的 x 平移（基线，带符号）
    q = np.zeros((4, 4), np.float64)
    q[0, 0] = 1.0
    q[0, 3] = -cx
    q[1, 1] = 1.0
    q[1, 3] = -cy
    q[2, 3] = f
    q[3, 2] = -1.0 / tx
    q[3, 3] = (cx - cxr) / tx
    return q


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth')

        # ---- 参数 ----
        self.declare_parameter('left_topic', '/camera/left/image_raw')
        self.declare_parameter('right_topic', '/camera/right/image_raw')
        self.declare_parameter('frame_id', 'camera_left_optical_frame')
        self.declare_parameter('scale', 0.5)          # 降采样比例（1.0=全尺寸）
        self.declare_parameter('num_disparities', 128)
        self.declare_parameter('block_size', 9)
        self.declare_parameter('depth_max', 5.0)
        self.declare_parameter('cloud_step', 2)       # 点云采样步长（1=全密度）
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('publish_disparity', True)
        self.declare_parameter('publish_cloud', True)

        left_topic = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        scale = float(self.get_parameter('scale').value)
        num_disp = (int(self.get_parameter('num_disparities').value) // 16) * 16
        block = int(self.get_parameter('block_size').value)
        if block % 2 == 0:
            block += 1
        self._depth_max = float(self.get_parameter('depth_max').value)
        self._cloud_step = max(1, int(self.get_parameter('cloud_step').value))
        self._pub_depth = bool(self.get_parameter('publish_depth').value)
        self._pub_disp = bool(self.get_parameter('publish_disparity').value)
        self._pub_cloud = bool(self.get_parameter('publish_cloud').value)

        # ---- 读标定，构造校正映射与 Q ----
        pkg_dir = get_package_share_directory('k7_camera')
        K1, D1, R1, P1, size = load_projection(os.path.join(pkg_dir, 'config', 'left.yaml'))
        K2, D2, R2, P2, _ = load_projection(os.path.join(pkg_dir, 'config', 'right.yaml'))

        # 缩放：P 的 fx/fy/cx/cy/Tx 同乘 scale，尺寸同乘 scale
        scaled_size = (int(size[0] * scale), int(size[1] * scale))
        P1s = P1.copy(); P2s = P2.copy()
        P1s[0, 0] *= scale; P1s[1, 1] *= scale
        P1s[0, 2] *= scale; P1s[1, 2] *= scale
        P2s[0, 0] *= scale; P2s[1, 1] *= scale
        P2s[0, 2] *= scale; P2s[1, 2] *= scale
        P2s[0, 3] *= scale

        self._mapL1, self._mapL2 = cv2.initUndistortRectifyMap(K1, D1, R1, P1s, scaled_size, cv2.CV_32FC1)
        self._mapR1, self._mapR2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2s, scaled_size, cv2.CV_32FC1)
        self._Q = build_q(P1s, P2s).astype(np.float32)
        self._f = float(P1s[0, 0])
        self._baseline = float(abs(P2s[0, 3]) / self._f)

        # ---- 匹配器 ----
        self._left_matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=block,
            P1=8 * block * block, P2=32 * block * block,
            disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=32, mode=cv2.STEREO_SGBM_MODE_SGBM)
        self._right_matcher = cv2.ximgproc.createRightMatcher(self._left_matcher)
        self._wls = cv2.ximgproc.createDisparityWLSFilter(self._left_matcher)
        self._wls.setLambda(80000.0)
        self._wls.setSigmaColor(1.3)

        self._bridge = CvBridge()

        # ---- 订阅（同步左右图）----
        left_sub = Subscriber(self, Image, left_topic)
        right_sub = Subscriber(self, Image, right_topic)
        self._sync = ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.2)
        self._sync.registerCallback(self._cb)

        # ---- 发布 ----
        self._depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 1)
        self._disp_pub = self.create_publisher(Image, '/camera/disparity/image', 1)
        self._cloud_pub = self.create_publisher(PointCloud2, '/camera/points2', 1)

        self.get_logger().info(
            f'stereo_depth 就绪: 订阅 {left_topic} + {right_topic}, '
            f'scale={scale}, 校正尺寸={scaled_size}, 基线={self._baseline * 1000:.1f}mm')

    def _cb(self, left_msg, right_msg):
        try:
            left = self._bridge.imgmsg_to_cv2(left_msg, 'rgb8')
            right = self._bridge.imgmsg_to_cv2(right_msg, 'rgb8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return

        # 校正（降采样后的校正图，remap 直接输出缩放尺寸）
        rectL = cv2.remap(left, self._mapL1, self._mapL2, cv2.INTER_LINEAR)
        rectR = cv2.remap(right, self._mapR1, self._mapR2, cv2.INTER_LINEAR)
        grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

        # SGBM + WLS
        dispL = self._left_matcher.compute(grayL, grayR).astype(np.float32) / 16.0
        dispR = self._right_matcher.compute(grayR, grayL).astype(np.float32) / 16.0
        disp = self._wls.filter(dispL, grayL, None, dispR)

        now = self.get_clock().now().to_msg()
        h, w = disp.shape

        # 深度图（32FC1, 米）
        if self._pub_depth:
            disp_safe = np.where(disp > 0.5, disp, 0.5)
            depth = (self._f * self._baseline / disp_safe).astype(np.float32)
            depth = np.clip(depth, 0.0, self._depth_max)
            depth_msg = self._bridge.cv2_to_imgmsg(depth, '32FC1')
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self._frame_id
            self._depth_pub.publish(depth_msg)

        # 视差图
        if self._pub_disp:
            disp_msg = self._bridge.cv2_to_imgmsg(disp.astype(np.float32), '32FC1')
            disp_msg.header.stamp = now
            disp_msg.header.frame_id = self._frame_id
            self._disp_pub.publish(disp_msg)

        # 点云
        if self._pub_cloud:
            xyz = cv2.reprojectImageTo3D(disp.astype(np.float32), self._Q)
            self._publish_cloud(xyz, rectL, now)

    def _publish_cloud(self, xyz, rectL, stamp):
        step = self._cloud_step
        ys, xs = np.mgrid[0:xyz.shape[0]:step, 0:xyz.shape[1]:step]
        z = xyz[ys, xs, 2]
        valid = np.isfinite(z) & (z > 0) & (z <= self._depth_max)
        ys, xs = ys[valid], xs[valid]
        if ys.size == 0:
            return

        x = xyz[ys, xs, 0]
        y = xyz[ys, xs, 1]
        z = z[valid]
        bgr = rectL[ys, xs]
        # RGB 打包进 float32（point_cloud2 的 rgb 约定）
        rgb = (bgr[:, 2].astype(np.float32) * 65536.0 +
               bgr[:, 1].astype(np.float32) * 256.0 +
               bgr[:, 0].astype(np.float32))

        cloud_arr = np.zeros(ys.size, dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
        cloud_arr['x'] = x
        cloud_arr['y'] = y
        cloud_arr['z'] = z
        cloud_arr['rgb'] = rgb

        header = rclpy.header.Header()
        header.stamp = stamp
        header.frame_id = self._frame_id
        cloud_msg = point_cloud2.create_cloud(header, FIELDS, cloud_arr)
        self._cloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
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
