"""Stereo depth convention; independent of ROS and the matcher."""
import numpy as np


def depth_from_disparity(disparity, min_disparity, numerator, cx_delta):
    # Z = (P_left[0,3] - P_right[0,3]) / (d - cx_left + cx_right).
    denominator = disparity-cx_delta
    valid = (disparity > min_disparity-0.5) & (np.abs(denominator) > 0.5)
    depth = np.full(disparity.shape, np.nan, dtype=np.float32)
    np.divide(numerator, denominator, out=depth, where=valid)
    depth[depth <= 0] = np.nan
    return depth


def obstacle_sectors(depth, p, intrinsics):
    fx, fy, cx, cy = intrinsics
    top, bottom = int(depth.shape[0]*p['roi_top']), int(depth.shape[0]*p['roi_bottom'])
    z = depth[top:bottom:3, ::3]
    yy, xx = np.mgrid[top:bottom:3, 0:depth.shape[1]:3]
    optical_x, optical_y = (xx-cx)/fx*z, (yy-cy)/fy*z
    cp, sp = np.cos(p['camera_pitch']), np.sin(p['camera_pitch'])
    forward, height = cp*z-sp*optical_y, p['camera_height']-sp*z-cp*optical_y
    cyaw, syaw = np.cos(p['camera_yaw']), np.sin(p['camera_yaw'])
    x, y = p['camera_x']+cyaw*forward+syaw*optical_x, syaw*forward-cyaw*optical_x
    angles, distances = np.arctan2(y, x), np.hypot(x, y)
    nominal = p['camera_yaw']-np.arctan2(xx-cx, fx)
    increment = 2*p['half_fov']/(p['sectors']-1)
    result = []
    for i in range(p['sectors']):
        center = -p['half_fov']+i*increment
        column = np.abs(nominal-center) <= increment/2
        measured = column & np.isfinite(z)
        sector = (np.abs(angles-center) <= increment/2) & np.isfinite(z) & (x > 0)
        obstacles = sector & (height >= p['obstacle_min_height']) & (height <= p['obstacle_max_height'])
        # Ground returns only prove clearance up to their range, never infinity.
        ground = sector & (height >= -0.05) & (height < p['obstacle_min_height'])
        visible = obstacles | ground
        count = int(np.count_nonzero(column))
        if (count < p['min_samples'] or np.count_nonzero(measured) < count*p['min_valid_fraction']
                or np.count_nonzero(visible) < p['min_samples']):
            result.append(float('nan'))
            continue
        distance = float(np.min(distances[visible]))
        result.append(max(p['range_min'], distance) if distance <= p['range_max'] else float('inf'))
    return result
