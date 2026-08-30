"""Shared right/inward bypass state and soft direction-consistency penalty."""

from __future__ import annotations

from .common_config import (
    SENSOR_MAX_RANGE,
    SIDE_CLEAR_DIST,
    SIDE_CLEAR_STEPS,
    SIDE_ENTER_DIST,
    W_SIDE_LOCK,
)


class FixedRightBypass:
    """Lock RIGHT only during an obstacle encounter; release after clear readings."""

    def __init__(self):
        self.active = False
        self.clear_count = 0

    def update(self, sensors):
        d_min = float(min(sensors.values()))
        detected = any(value < SENSOR_MAX_RANGE - 1e-9 for value in sensors.values())
        # Sensor readings are robot-center to obstacle-surface hit distances.
        # A remembered hazard point alone must not start the direction lock.
        entering = d_min < SIDE_ENTER_DIST
        if not self.active and entering:
            self.active = True
            self.clear_count = 0

        if self.active:
            clear = (not detected) or d_min > SIDE_CLEAR_DIST
            self.clear_count = self.clear_count + 1 if clear else 0
            if self.clear_count >= SIDE_CLEAR_STEPS:
                self.active = False
                self.clear_count = 0
        return self.active


def right_bypass_cost(omega, active):
    """Positive omega is left turn, hence penalized during a RIGHT bypass."""
    return W_SIDE_LOCK * max(0.0, float(omega)) ** 2 if active else 0.0
