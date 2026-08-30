"""Unknown-size obstacle model based on one filtered representative hit point."""

from __future__ import annotations

import numpy as np

from .common_config import (
    HAZARD_FILTER_ALPHA,
    SENSOR_ANGLE_OFFSETS,
    SENSOR_MAX_RANGE,
    VIRTUAL_OBS_HOLD_STEPS,
)


def build_virtual_hazard_points(x, y, theta, sensors, previous_points):
    weighted_sum = np.zeros(2, dtype=float)
    weight_sum = 0.0
    sources = []
    origin = np.array([x, y], dtype=float)

    for name, distance in sensors.items():
        if distance >= SENSOR_MAX_RANGE - 1e-9:
            continue
        ray_angle = theta + SENSOR_ANGLE_OFFSETS[name]
        ray = np.array([np.cos(ray_angle), np.sin(ray_angle)], dtype=float)
        hit_point = origin + float(distance) * ray
        weight = 1.0 / max(float(distance), 1e-6)
        weighted_sum += weight * hit_point
        weight_sum += weight
        sources.append(name)

    previous_point = None
    previous_age = 0
    if previous_points:
        previous_point = np.asarray(previous_points[0]["point"], dtype=float)
        previous_age = int(previous_points[0].get("age", 0))

    if weight_sum > 0.0:
        representative = weighted_sum / weight_sum
        if previous_point is None:
            filtered = representative
        else:
            filtered = HAZARD_FILTER_ALPHA * previous_point + (1.0 - HAZARD_FILTER_ALPHA) * representative
        return [{"point": filtered, "age": 0, "source": "+".join(sources)}]

    if previous_point is not None and previous_age + 1 < VIRTUAL_OBS_HOLD_STEPS:
        return [{"point": previous_point, "age": previous_age + 1, "source": "held"}]
    return []
