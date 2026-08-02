"""One shared fuzzy-risk rule set for V1 logging and V2-V5 control modules."""

from __future__ import annotations

import numpy as np

from .common_config import (
    POINT_COLLISION_DIST,
    POINT_SAFE_DIST,
    POINT_WARNING_DIST,
    RISK_HIGH_ENTER,
    RISK_HIGH_EXIT,
    RISK_HIGH_THRESHOLD,
    RISK_LEVEL_HOLD_STEPS,
    RISK_LOW_THRESHOLD,
    RISK_MEDIUM_ENTER,
    RISK_MEDIUM_EXIT,
    SENSOR_MAX_RANGE,
)


def clamp01(value):
    return float(np.clip(value, 0.0, 1.0))


def risk_level_from_value(risk):
    if risk < RISK_LOW_THRESHOLD:
        return "LOW"
    if risk < RISK_HIGH_THRESHOLD:
        return "MEDIUM"
    return "HIGH"


def compute_fuzzy_risk(sensors, prev_d_min, dt):
    """Compute risk from ray-hit surface distance, never true center distance."""
    d_min = float(min(sensors.values()))
    closing_rate = 0.0 if prev_d_min is None else float((prev_d_min - d_min) / dt)
    if all(distance >= SENSOR_MAX_RANGE - 1e-9 for distance in sensors.values()):
        return 0.0, "LOW", d_min, closing_rate

    if d_min <= POINT_SAFE_DIST:
        near = 1.0
    elif d_min < POINT_WARNING_DIST:
        near = (POINT_WARNING_DIST - d_min) / (POINT_WARNING_DIST - POINT_SAFE_DIST)
    else:
        near = 0.0

    if d_min <= POINT_COLLISION_DIST:
        medium = 0.0
    elif d_min < POINT_SAFE_DIST:
        medium = (d_min - POINT_COLLISION_DIST) / (POINT_SAFE_DIST - POINT_COLLISION_DIST)
    elif d_min <= POINT_WARNING_DIST:
        medium = 1.0
    elif d_min < 1.40:
        medium = (1.40 - d_min) / (1.40 - POINT_WARNING_DIST)
    else:
        medium = 0.0

    if d_min <= POINT_WARNING_DIST:
        far = 0.0
    elif d_min < 1.40:
        far = (d_min - POINT_WARNING_DIST) / (1.40 - POINT_WARNING_DIST)
    else:
        far = 1.0

    receding = 1.0 if closing_rate <= -0.05 else clamp01(-closing_rate / 0.05)
    stable = clamp01(1.0 - abs(closing_rate) / 0.05)
    approaching = 1.0 if closing_rate >= 0.05 else clamp01(closing_rate / 0.05)
    rules = [
        (near, 0.85),
        (min(medium, approaching), 0.85),
        (min(medium, stable), 0.50),
        (min(medium, receding), 0.35),
        (min(far, approaching), 0.50),
        (min(far, stable), 0.20),
        (min(far, receding), 0.20),
    ]
    strength_sum = sum(strength for strength, _ in rules)
    risk = 0.0 if strength_sum <= 1e-12 else sum(s * value for s, value in rules) / strength_sum
    risk = clamp01(risk)
    return risk, risk_level_from_value(risk), d_min, closing_rate


class StableRiskLevel:
    """LOW/MEDIUM/HIGH hysteresis with a six-cycle minimum hold time."""

    def __init__(self):
        self.level = "LOW"
        self.hold_remaining = 0
        self.switch_count = 0

    def update(self, risk):
        previous = self.level
        emergency = risk >= 0.95
        if self.hold_remaining > 0 and not emergency:
            self.hold_remaining -= 1
            return self.level, False

        if self.level == "LOW":
            if emergency:
                self.level = "HIGH"
            elif risk >= RISK_MEDIUM_ENTER:
                self.level = "MEDIUM"
        elif self.level == "MEDIUM":
            if risk >= RISK_HIGH_ENTER:
                self.level = "HIGH"
            elif risk <= RISK_MEDIUM_EXIT:
                self.level = "LOW"
        elif risk <= RISK_HIGH_EXIT:
            self.level = "MEDIUM"

        changed = self.level != previous
        if changed:
            self.switch_count += 1
            self.hold_remaining = RISK_LEVEL_HOLD_STEPS
        return self.level, changed
