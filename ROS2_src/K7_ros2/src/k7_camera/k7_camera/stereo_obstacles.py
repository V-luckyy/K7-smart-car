"""Single-process stereo matching and bounded obstacle sectors; no point cloud."""
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, LaserScan
from .stereo_geometry import depth_from_disparity, obstacle_sectors


def calibration(path):
    with open(path, encoding='utf-8') as stream:
        doc = yaml.safe_load(stream)
    matrices = [np.array(doc[key]['data'], dtype=np.float64).reshape(shape)
                for key, shape in [('camera_matrix', (3, 3)),
                                   ('distortion_coefficients', (-1,)),
                                   ('rectification_matrix', (3, 3)),
                                   ('projection_matrix', (3, 4))]]
    return (*matrices, (doc['image_width'], doc['image_height']))


class StereoObstacles(Node):
    def __init__(self):
        super().__init__('stereo_obstacles')
        defaults = dict(scale=0.25, rate=10.0, num_disparities=64, block_size=5,
            sectors=21, half_fov=0.50, range_min=0.05, range_max=4.0,
            min_valid_fraction=0.6, min_samples=8, sensor_timeout=0.4,
            roi_top=0.25, roi_bottom=0.65, camera_x=0.0, camera_height=0.20,
            camera_pitch=0.0, camera_yaw=0.0, obstacle_min_height=0.05,
            obstacle_max_height=0.70, calibration_confirmed=False,
            base_frame='base_link', optical_frame='camera_left_optical_frame',
            publish_depth=False, opencv_threads=2,
            left_calibration='', right_calibration='')
        descriptor = ParameterDescriptor(read_only=True)
        self.p = {key: self.declare_parameter(key, value, descriptor).value for key, value in defaults.items()}
        p = self.p
        if not (0 < p['scale'] <= 1 and p['rate'] > 0 and p['sectors'] >= 3 and
                0 < p['half_fov'] < math.pi/2 and 0 < p['range_min'] < p['range_max'] and
                0 <= p['roi_top'] < p['roi_bottom'] <= 1 and
                0 < p['min_valid_fraction'] <= 1 and p['min_samples'] > 0 and
                p['num_disparities'] > 0 and p['num_disparities'] % 16 == 0 and
                p['block_size'] >= 3 and p['block_size'] % 2 == 1):
            raise ValueError('invalid stereo geometry, ROI or matching limits')
        cv2.setNumThreads(p['opencv_threads'])
        root = os.path.join(get_package_share_directory('k7_camera'), 'config')
        kl, dl, rl, pl, self.size = calibration(p['left_calibration'] or os.path.join(root, 'left.yaml'))
        kr, dr, rr, pr, right_size = calibration(p['right_calibration'] or os.path.join(root, 'right.yaml'))
        if right_size != self.size:
            raise ValueError('stereo calibration resolutions differ')
        pl[:2, :] *= p['scale']
        pr[:2, :] *= p['scale']
        size = tuple(int(v*p['scale']) for v in self.size)
        self.lmap = cv2.initUndistortRectifyMap(kl, dl, rl, pl, size, cv2.CV_32FC1)
        self.rmap = cv2.initUndistortRectifyMap(kr, dr, rr, pr, size, cv2.CV_32FC1)
        self.fx, self.fy, self.cx, self.cy = pl[0, 0], pl[1, 1], pl[0, 2], pl[1, 2]
        self.numerator = pl[0, 3]-pr[0, 3]
        self.cx_delta = pl[0, 2]-pr[0, 2]
        if abs(self.numerator) < 1e-6:
            raise ValueError('zero stereo baseline')
        # A positive right P[0,3] implies negative left-minus-right disparity.
        self.min_disp = -p['num_disparities'] if self.numerator < 0 else 0
        block = p['block_size']
        self.matcher = cv2.StereoSGBM_create(minDisparity=self.min_disp,
            numDisparities=p['num_disparities'], blockSize=block,
            P1=8*block*block, P2=32*block*block, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=60, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM)
        self.bridge = CvBridge()
        self.last = -math.inf
        self.scan_pub = self.create_publisher(LaserScan, 'perception/scan', qos_profile_sensor_data)
        self.depth_pub = (self.create_publisher(Image, 'camera/depth/image_raw', qos_profile_sensor_data)
                          if p['publish_depth'] else None)
        self.create_subscription(Image, 'camera/image_raw', self.image_cb, qos_profile_sensor_data)
        if not p['calibration_confirmed']:
            self.get_logger().warn('Drive inhibited: verify stereo order, baseline units and camera mounting first')

    def image_cb(self, msg):
        now = time.monotonic()
        if now-self.last < 1/self.p['rate']:
            return
        self.last = now
        p = self.p
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = msg.header.stamp, p['base_frame']
        scan.angle_min, scan.angle_max = -p['half_fov'], p['half_fov']
        scan.angle_increment = 2*p['half_fov']/(p['sectors']-1)
        scan.range_min, scan.range_max = p['range_min'], p['range_max']
        scan.scan_time = 1/p['rate']
        scan.ranges = [float('nan')]*p['sectors']
        age = self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if not -0.05 <= age <= p['sensor_timeout']:
            self.scan_pub.publish(scan)
            return
        try:
            if (msg.width, msg.height) != (2*self.size[0], self.size[1]):
                raise ValueError('capture size must equal twice calibrated eye width and calibrated height')
            gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            width = self.size[0]
            left = cv2.remap(gray[:, :width], *self.lmap, cv2.INTER_LINEAR)
            right = cv2.remap(gray[:, width:], *self.rmap, cv2.INTER_LINEAR)
            disparity = self.matcher.compute(left, right).astype(np.float32)/16
            depth = depth_from_disparity(disparity, self.min_disp, self.numerator, self.cx_delta)
            if self.depth_pub and self.depth_pub.get_subscription_count():
                image = self.bridge.cv2_to_imgmsg(depth, '32FC1')
                image.header.stamp, image.header.frame_id = msg.header.stamp, p['optical_frame']
                self.depth_pub.publish(image)
            if p['calibration_confirmed']:
                scan.ranges = obstacle_sectors(depth, p, (self.fx, self.fy, self.cx, self.cy))
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = StereoObstacles()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
