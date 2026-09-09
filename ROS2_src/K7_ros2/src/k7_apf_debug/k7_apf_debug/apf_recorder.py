#!/usr/bin/env python3
# coding=utf-8
"""APF + Stanley 实车调参 —— /apf_debug 记录成 CSV。

前置：STM32 烧含 0xFB 调试帧的固件并自跑 APF 圆；K7 端以"只收不发"模式
跑 k7_serial_node（enable_downlink:=false）把 0xFB 帧发布成 /apf_debug。
本节点订阅 /apf_debug，逐帧落盘 CSV，并在 Python 侧算好 Stanley 调参最关心的
两个量（相对参考圆）：
    cte    = 到圆心距离 − R                    （横向误差，正=圆外）
    psi_e  = 圆切线方向角 − 车头朝向 θ（wrap 到 ±π）  （航向误差）

CSV 列：
    t, x, y, theta, cte, psi_e,
    v_cmd, w_cmd, w_stanley, w_apf, v_act, w_act,
    d_front, d_left, d_right

用法：
    ros2 run k7_apf_debug apf_recorder            # 默认写到 ~/apf_log/apf_<时间>.csv
    ros2 run k7_apf_debug apf_recorder --ros-args \
        -p out_dir:=/home/kickpi/logs -p circle_radius:=0.6
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from k7_msgs.msg import ApfDebug

CSV_HEADER = ['t', 'x', 'y', 'theta', 'cte', 'psi_e',
              'v_cmd', 'w_cmd', 'w_stanley', 'w_apf', 'v_act', 'w_act',
              'd_front', 'd_left', 'd_right']


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ApfRecorder(Node):
    def __init__(self):
        super().__init__('apf_recorder')

        self.declare_parameter('apf_topic', 'apf_debug')   # 相对话题名，namespace 下自动落位
        self.declare_parameter('out_dir', os.path.join(os.path.expanduser('~'), 'apf_log'))
        self.declare_parameter('file_prefix', 'apf_')
        # 参考路径：'circle' 参考圆 / 'line' 参考直线 y=0 沿 +x
        # （须与 stanley.h 里当前硬编码路径一致：圆→STANLEY_CIRCLE_*，直线→STANLEY_LINE_LEN/HEADING）
        self.declare_parameter('reference', 'circle')
        self.declare_parameter('circle_cx', 0.0)
        self.declare_parameter('circle_cy', 0.0)
        self.declare_parameter('circle_radius', 0.6)

        self._ref = self.get_parameter('reference').value
        self._cx = float(self.get_parameter('circle_cx').value)
        self._cy = float(self.get_parameter('circle_cy').value)
        self._R = float(self.get_parameter('circle_radius').value)

        out_dir = os.path.expanduser(self.get_parameter('out_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(out_dir,
                            f"{self.get_parameter('file_prefix').value}{stamp}.csv")
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(CSV_HEADER)
        self._n = 0
        self._t0 = None

        topic = self.get_parameter('apf_topic').value
        self.create_subscription(ApfDebug, topic, self._on_apf, 20)
        if self._ref == 'line':
            self.get_logger().info(
                f'记录开始: <{topic}> -> {path}  (参考直线: y=0 沿 +x, 算 cte/psi_e 用直线式)')
        else:
            self.get_logger().info(
                f'记录开始: <{topic}> -> {path}  (参考圆: 圆心({self._cx},{self._cy}) R={self._R})')

    def _on_apf(self, msg: ApfDebug):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0

        # Stanley 相对参考路径的横向误差与航向误差（与 stanley.c 同约定）
        if self._ref == 'line':
            # 参考直线 y=0 沿 +x：cte=-y（偏左+放大、需右回），psi_e = 0 - theta
            cte = -msg.y
            psi_e = wrap_pi(-msg.theta)
        else:
            dx = msg.x - self._cx
            dy = msg.y - self._cy
            cte = math.hypot(dx, dy) - self._R             # 正=圆外
            tangent = math.atan2(-dy, dx)                  # 逆时针圆切线方向角
            psi_e = wrap_pi(tangent - msg.theta)           # 航向误差

        self._w.writerow([
            f'{rel:.4f}', f'{msg.x:.4f}', f'{msg.y:.4f}', f'{msg.theta:.4f}',
            f'{cte:.4f}', f'{psi_e:.4f}',
            f'{msg.v_cmd:.4f}', f'{msg.w_cmd:.4f}',
            f'{msg.w_stanley:.4f}', f'{msg.w_apf:.4f}',
            f'{msg.v_act:.4f}', f'{msg.w_act:.4f}',
            f'{msg.d_front:.3f}', f'{msg.d_left:.3f}', f'{msg.d_right:.3f}',
        ])
        self._f.flush()
        self._n += 1
        if self._n % 50 == 0:                              # 每 2.5s 提示一次
            self.get_logger().info(f'已记录 {self._n} 帧 (x={msg.x:.2f},y={msg.y:.2f})')

    def __del__(self):
        try:
            self._f.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ApfRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._f.close()
        except Exception:
            pass
        node.get_logger().info(f'记录结束，共 {node._n} 帧')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
