#!/usr/bin/env python3
# coding=utf-8
"""双目深度图测试脚本（离线，不依赖 ROS）。

用 k7_camera/config/{left,right}.yaml 的标定参数（内参/畸变/校正矩阵/投影矩阵），
对 calibration_images 里的左右一对图像做去畸变 + 极线校正，StereoBM/SGBM 计算视差，
再转成深度图保存。

用法（在 RK3576 根目录或任意位置）:
    python ROS2_src/K7_ros2/src/k7_camera/scripts/generate_depth.py            # 第 0 对
    python .../generate_depth.py --index 3 --sgbm                             # 第 3 对 + SGBM
    python .../generate_depth.py --depth-max 3.0                              # 深度色标上限 3m
"""

import argparse
import os

import cv2
import numpy as np
import yaml


def load_calib(path):
    """读 camera_calibration 风格的 yaml，返回 K, D, R, P, (w, h)。"""
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    R = np.array(d['rectification_matrix']['data'], dtype=np.float64).reshape(3, 3)
    P = np.array(d['projection_matrix']['data'], dtype=np.float64).reshape(3, 4)
    size = (int(d['image_width']), int(d['image_height']))
    return K, D, R, P, size


def main():
    ap = argparse.ArgumentParser(description='双目深度图测试')
    ap.add_argument('--index', type=int, default=0, help='图像对序号（0..N-1）')
    ap.add_argument('--sgbm', action='store_true', help='用 SGBM（更慢更平滑），默认 StereoBM')
    ap.add_argument('--depth-max', type=float, default=5.0, help='深度图色标上限 (m)')
    ap.add_argument('--out', default=None, help='深度图输出路径（默认脚本旁 depth.png）')
    ap.add_argument('--disparity-out', default=None, help='视差图输出路径')
    args = ap.parse_args()

    # 相对脚本位置定位 RK3576 根目录
    base = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(base, '..', '..', '..', '..', '..'))
    config_dir = os.path.join(root, 'ROS2_src', 'K7_ros2', 'src', 'k7_camera', 'config')
    img_dir = os.path.join(root, 'calibration_images')

    K1, D1, R1, P1, size = load_calib(os.path.join(config_dir, 'left.yaml'))
    K2, D2, R2, P2, _ = load_calib(os.path.join(config_dir, 'right.yaml'))

    # 焦距与基线（从投影矩阵提取：P2[0,3] = -f*Tx，物理基线 = |Tx|）
    f = float(P1[0, 0])
    baseline = float(abs(P2[0, 3]) / f)
    print(f'焦距 f={f:.2f} px, 基线 baseline={baseline * 1000:.1f} mm')

    # 读左右图（灰度）
    left_path = os.path.join(img_dir, 'left', f'left_{args.index:03d}.png')
    right_path = os.path.join(img_dir, 'right', f'right_{args.index:03d}.png')
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
    if imgL is None or imgR is None:
        raise SystemExit(f'读图失败: {left_path} / {right_path}')
    print(f'图像: {os.path.basename(left_path)} / {os.path.basename(right_path)}  {imgL.shape[1]}x{imgL.shape[0]}')

    # 去畸变 + 极线校正（用各自的 rectify/projection 矩阵）
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1)
    rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)

    # 视差（numDisparities 必须能被 16 整除）
    if args.sgbm:
        stereo = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=9,
            P1=8 * 3 * 9 * 9, P2=32 * 3 * 9 * 9,
            disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=32)
    else:
        stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)
    disparity = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    # 深度 = f * baseline / disparity
    disparity_safe = np.where(disparity > 0.5, disparity, 0.5)
    depth = f * baseline / disparity_safe
    depth = np.clip(depth, 0.0, args.depth_max)

    # 可视化
    disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

    out = args.out or os.path.join(base, 'depth.png')
    disp_out = args.disparity_out or os.path.join(base, 'disparity.png')
    cv2.imwrite(disp_out, disp_norm)
    cv2.imwrite(out, depth_colormap)

    valid = disparity[disparity > 0.5]
    print(f'有效视差像素占比: {100.0 * valid.size / disparity.size:.1f}%')
    if valid.size:
        d = f * baseline / valid
        d = d[d <= args.depth_max]
        print(f'深度范围: {d.min():.2f} ~ {d.max():.2f} m')
    print(f'已保存: 视差图={disp_out}, 深度图={out}')


if __name__ == '__main__':
    main()
