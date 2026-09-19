#!/usr/bin/env python3
# coding=utf-8
"""双目立体匹配 + 视差图（SGBM + WLS 滤波）—— 参考丁先生《保姆级双目重建原理及代码》。

前置：stereo.py 已完成标定，config/left.yaml、right.yaml 含校正参数（rectification/projection 矩阵）。
流程：读标定参数 → remap 极线校正 → SGBM 左/右视差 → WLS 滤波后处理 → 视差图 + 深度图。

产出（默认输出到脚本目录）：
    disparity_sgbm.png   —— 原始 SGBM 视差图（归一化可视化）
    disparity_wls.png    —— WLS 滤波后视差图（更平滑、去噪）
    depth_wls.png        —— 深度图（JET 色标，由 WLS 视差换算）

用法（在 RK3576 根目录或任意位置）：
    python ROS2_src/K7_ros2/src/k7_camera/scripts/stereo_match.py                # 第 0 对标定图
    python .../stereo_match.py --left L.png --right R.png                        # 指定任意左右实景图
    python .../stereo_match.py --num-disparities 64 --depth-max 3.0              # 调参
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')


def load_calib(path):
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], np.float64)
    R = np.array(d['rectification_matrix']['data'], np.float64).reshape(3, 3)
    P = np.array(d['projection_matrix']['data'], np.float64).reshape(3, 4)
    size = (int(d['image_width']), int(d['image_height']))
    return K, D, R, P, size


def main():
    ap = argparse.ArgumentParser(description='双目立体匹配 + 视差图（SGBM + WLS）')
    ap.add_argument('--left', default=None, help='左图路径（默认 calibration_images/left/left_000.png）')
    ap.add_argument('--right', default=None, help='右图路径')
    ap.add_argument('--config-dir', default=None, help='标定 config 目录（默认 k7_camera/config）')
    ap.add_argument('--out-dir', default=None, help='输出目录（默认脚本所在目录）')
    ap.add_argument('--num-disparities', type=int, default=128, help='视差搜索范围（16 的倍数）')
    ap.add_argument('--block-size', type=int, default=9, help='SGBM 块大小（奇数）')
    ap.add_argument('--depth-max', type=float, default=5.0, help='深度图色标上限 (m)')
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(script_dir, '..', '..', '..', '..', '..'))

    config_dir = args.config_dir or os.path.join(
        root, 'ROS2_src', 'K7_ros2', 'src', 'k7_camera', 'config')
    img_dir = os.path.join(root, 'calibration_images')
    left_path = args.left or os.path.join(img_dir, 'left', 'left_000.png')
    right_path = args.right or os.path.join(img_dir, 'right', 'right_000.png')
    out_dir = args.out_dir or script_dir

    # ---- 1. 读标定 + 校正映射 ----
    K1, D1, R1, P1, size = load_calib(os.path.join(config_dir, 'left.yaml'))
    K2, D2, R2, P2, _ = load_calib(os.path.join(config_dir, 'right.yaml'))

    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
    if imgL is None or imgR is None:
        raise SystemExit(f'读图失败: {left_path} / {right_path}')

    mapL1, mapL2 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)
    mapR1, mapR2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1)
    rectL = cv2.remap(imgL, mapL1, mapL2, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapR1, mapR2, cv2.INTER_LINEAR)

    # ---- 2. SGBM 左视差 + 右视差（WLS 需要一对）----
    numDisp = (args.num_disparities // 16) * 16
    block = args.block_size if args.block_size % 2 == 1 else args.block_size + 1
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=numDisp, blockSize=block,
        P1=8 * 1 * block * block, P2=32 * 1 * block * block,
        disp12MaxDiff=1, uniquenessRatio=10,
        speckleWindowSize=100, speckleRange=32, mode=cv2.STEREO_SGBM_MODE_SGBM)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    dispL_raw = left_matcher.compute(rectL, rectR).astype(np.float32) / 16.0   # int16 -> 实际视差(px)
    dispR_raw = right_matcher.compute(rectR, rectL).astype(np.float32) / 16.0

    # ---- 3. WLS 滤波 ----
    wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls.setLambda(80000.0)
    wls.setSigmaColor(1.3)
    disp_wls = wls.filter(dispL_raw, rectL, None, dispR_raw)   # float32，实际视差(px)

    # ---- 4. 深度：depth = f * baseline / disparity ----
    f = float(P1[0, 0])
    baseline = float(abs(P2[0, 3]) / f)
    disp_safe = np.where(disp_wls > 0.5, disp_wls, 0.5)
    depth = f * baseline / disp_safe
    depth = np.clip(depth, 0.0, args.depth_max)
    print(f'焦距 f={f:.1f}px, 基线 baseline={baseline * 1000:.1f}mm')

    # ---- 5. 可视化 + 保存 ----
    def norm_vis(disp, vmax=None):
        vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return cv2.applyColorMap(vis, cv2.COLORMAP_JET)

    disp_raw_vis = norm_vis(dispL_raw)
    disp_wls_vis = norm_vis(disp_wls)
    depth_vis = norm_vis(depth)

    p_raw = os.path.join(out_dir, 'disparity_sgbm.png')
    p_wls = os.path.join(out_dir, 'disparity_wls.png')
    p_depth = os.path.join(out_dir, 'depth_wls.png')
    cv2.imwrite(p_raw, disp_raw_vis)
    cv2.imwrite(p_wls, disp_wls_vis)
    cv2.imwrite(p_depth, depth_vis)

    valid = disp_wls[disp_wls > 0.5]
    print(f'有效视差像素占比: {100.0 * valid.size / disp_wls.size:.1f}%')
    if valid.size:
        d = f * baseline / valid
        d = d[d <= args.depth_max]
        print(f'深度范围: {d.min():.2f} ~ {d.max():.2f} m')
    print(f'已保存:\n  {p_raw}\n  {p_wls}\n  {p_depth}')


if __name__ == '__main__':
    main()
