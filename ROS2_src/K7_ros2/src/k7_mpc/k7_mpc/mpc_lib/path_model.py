"""Reference-path indexing and unicycle kinematics shared by all versions.

实车移植说明：`generate_eight_path` 增加可选传参（默认仍取本模块常量，
仿真行为不变），实车路径幅度由 mpc_config.PATH_A 指定。
"""

from __future__ import annotations

import numpy as np

from .common_config import (
    INTEGRATION_SUBSTEPS,
    PATH_A,
    PATH_CYCLES,
    PATH_POINTS,
    REFERENCE_SEARCH_WINDOW,
)


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def generate_eight_path(a=None, points=None, cycles=None):
    a = PATH_A if a is None else float(a)
    points = PATH_POINTS if points is None else int(points)
    cycles = PATH_CYCLES if cycles is None else int(cycles)
    t = np.linspace(0.0, 2.0 * np.pi * cycles, points)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    dx = a * np.cos(t)
    dy = a * (np.cos(t) ** 2 - np.sin(t) ** 2)
    theta = np.unwrap(np.arctan2(dy, dx))
    ds = np.hypot(np.diff(x), np.diff(y))
    s = np.r_[0.0, np.cumsum(ds)]
    return x, y, theta, s


def nearest_forward_index(x, y, ref_x, ref_y, last_idx):
    start = int(max(0, last_idx))
    end = min(len(ref_x) - 1, start + REFERENCE_SEARCH_WINDOW)
    dx = ref_x[start : end + 1] - x
    dy = ref_y[start : end + 1] - y
    return start + int(np.argmin(dx * dx + dy * dy))


def reference_at_index(ref_x, ref_y, ref_theta, idx):
    safe_idx = int(np.clip(int(idx), 0, len(ref_x) - 1))
    return ref_x[safe_idx], ref_y[safe_idx], ref_theta[safe_idx]


def ref_index_by_distance(ref_s, base_idx, distance_ahead):
    target_s = ref_s[int(base_idx)] + float(distance_ahead)
    idx = int(np.searchsorted(ref_s, target_s, side="left"))
    return min(idx, len(ref_s) - 1)


def step_unicycle(x, y, theta, v, omega, dt):
    sub_dt = float(dt) / INTEGRATION_SUBSTEPS
    for _ in range(INTEGRATION_SUBSTEPS):
        theta = wrap_angle(theta + omega * sub_dt)
        x = x + v * np.cos(theta) * sub_dt
        y = y + v * np.sin(theta) * sub_dt
    return float(x), float(y), float(theta)
