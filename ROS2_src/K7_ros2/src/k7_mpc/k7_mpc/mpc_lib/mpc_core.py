"""Unified continuous SLSQP MPC engine for the progressive V1-V5 chain.

实车移植说明（2026-07-26）：相对仿真 r04 原文件仅删除 `run_simulation()` 及
其专用 import（environment.true_center_distance / sensors.simulate_three_ray_sensors /
generate_eight_path / reference_at_index / MAX_STEPS / FINISH_INDEX_MARGIN），
控制器算法本体逐字未改。
"""

from __future__ import annotations

import time

import numpy as np
from scipy.optimize import minimize

from .common_config import (
    DOMEGA_MAX,
    DT_CONTROL,
    DV_MAX,
    GRID_RESET_LAST_CONTROL_BLEND,
    GRID_RESET_NOMINAL_BLEND,
    INITIAL_REF_INDEX,
    OMEGA_MAX,
    OMEGA_MIN,
    OMEGA_REF,
    POINT_COLLISION_DIST,
    POINT_SAFE_DIST,
    POINT_WARNING_DIST,
    RISK_FILTER_ALPHA,
    SENSOR_MAX_RANGE,
    SOLVER_FEASIBILITY_TOL,
    SOLVER_FTOL,
    SOLVER_MAXITER,
    SOLVER_METHOD,
    V_MAX,
    V_MIN,
    V_PATH_PROGRESS,
    V_REF,
    W_DELTA_OMEGA,
    W_DELTA_V,
    W_HEADING,
    W_OBS_SAFE,
    W_OBS_WARN,
    W_OMEGA,
    W_SPEED,
    W_TERMINAL,
    W_TRACK,
    WARM_START_INITIAL_BLEND,
    WARM_START_NOMINAL_BLEND,
    WARM_START_POST_NOMINAL_BLEND,
    WARM_START_SHIFT_BLEND,
)
from .fixed_right_bypass import FixedRightBypass, right_bypass_cost
from .fuzzy_risk import StableRiskLevel, clamp01, compute_fuzzy_risk
from .hazard_points import build_virtual_hazard_points
from .path_model import (
    nearest_forward_index,
    ref_index_by_distance,
    step_unicycle,
    wrap_angle,
)


def obstacle_cost(px, py, hazard_points, obs_warn_weight, obs_safe_weight):
    """Hazard-point cost with no true obstacle center or radius input."""
    total = 0.0
    for item in hazard_points or []:
        point = np.asarray(item["point"], dtype=float)
        distance = float(np.hypot(px - point[0], py - point[1]))
        if distance <= POINT_COLLISION_DIST:
            ratio = (POINT_COLLISION_DIST - distance) / max(POINT_COLLISION_DIST, 1e-9)
            total += 1e6 + 1e6 * ratio * ratio
        elif distance < POINT_SAFE_DIST:
            ratio = (POINT_SAFE_DIST - distance) / (POINT_SAFE_DIST - POINT_COLLISION_DIST)
            total += obs_safe_weight * ratio * ratio
        elif distance < POINT_WARNING_DIST:
            ratio = (POINT_WARNING_DIST - distance) / (POINT_WARNING_DIST - POINT_SAFE_DIST)
            total += obs_warn_weight * ratio * ratio
    return float(total)


def select_horizon(version, control_risk_level):
    if not version["dynamic_horizon"]:
        return int(version["fixed_np"]), int(version["fixed_nc"])
    np_horizon, nc_horizon = version["horizon_map"][control_risk_level]
    return int(np_horizon), int(nc_horizon)


def build_prediction_dts(version, np_horizon, control_risk_level):
    """Only V4/V5 use 1x/2x DT_CONTROL; no 0.21 s step is permitted."""
    if not version["nonuniform_time_horizon"]:
        return np.full(np_horizon, DT_CONTROL, dtype=float)
    multipliers = np.asarray(
        version["prediction_dt_multipliers"][control_risk_level], dtype=float
    )
    if len(multipliers) != np_horizon:
        raise ValueError(f"{control_risk_level} prediction dt length does not equal NP")
    if np.any(multipliers < 1.0) or not np.allclose(multipliers, np.round(multipliers)):
        raise ValueError("Prediction dt must be an integer multiple of DT_CONTROL")
    if np.any(multipliers > 2.0):
        raise ValueError("This revision permits only 0.07 s and 0.14 s prediction steps")
    return multipliers * DT_CONTROL


def prediction_pattern_id(version, prediction_dts, control_risk_level):
    if not version["nonuniform_time_horizon"]:
        return "UNIFORM_1X"
    multipliers = np.rint(np.asarray(prediction_dts) / DT_CONTROL).astype(int)
    groups = []
    start = 0
    for index in range(1, len(multipliers) + 1):
        if index == len(multipliers) or multipliers[index] != multipliers[start]:
            groups.append(f"{multipliers[start]}Xx{index - start}")
            start = index
    return f"{control_risk_level}:" + "+".join(groups)


def dynamic_obstacle_weights(version, risk):
    if not version["dynamic_weight"]:
        return float(W_OBS_WARN), float(W_OBS_SAFE)
    mapping = version["dynamic_weight_map"]
    return (
        float(mapping["warn_base"] + mapping["warn_gain"] * risk),
        float(mapping["safe_base"] + mapping["safe_gain"] * risk),
    )


def predict_trajectory(state, control_seq, np_horizon, nc_horizon, prediction_dts):
    px, py, ptheta = state
    controls = np.asarray(control_seq, dtype=float).reshape(nc_horizon, 2)
    prediction = []
    for k in range(np_horizon):
        v, omega = controls[k] if k < nc_horizon else controls[-1]
        px, py, ptheta = step_unicycle(px, py, ptheta, v, omega, prediction_dts[k])
        prediction.append((px, py, ptheta, float(v), float(omega)))
    return prediction


def build_prediction_indices(ref_s, ref_idx, prediction_dts):
    elapsed = np.cumsum(np.asarray(prediction_dts, dtype=float))
    return np.asarray(
        [ref_index_by_distance(ref_s, ref_idx, V_PATH_PROGRESS * value) for value in elapsed],
        dtype=int,
    )


def mpc_cost(
    control_seq,
    state,
    ref_x,
    ref_y,
    ref_theta,
    pred_indices,
    hazard_points,
    last_control,
    np_horizon,
    nc_horizon,
    prediction_dts,
    obs_warn_weight,
    obs_safe_weight,
    right_bypass_active,
):
    prediction = predict_trajectory(
        state, control_seq, np_horizon, nc_horizon, prediction_dts
    )
    controls = np.asarray(control_seq, dtype=float).reshape(nc_horizon, 2)
    previous_v, previous_omega = map(float, last_control)
    total = 0.0
    for k, (px, py, ptheta, v, omega) in enumerate(prediction):
        ref_idx = pred_indices[k]
        # This is the only prediction-dt stage-cost scaling in the controller.
        scale = float(prediction_dts[k] / DT_CONTROL)
        track_error_sq = (px - ref_x[ref_idx]) ** 2 + (py - ref_y[ref_idx]) ** 2
        heading_error_sq = wrap_angle(ptheta - ref_theta[ref_idx]) ** 2
        stage = W_TRACK * track_error_sq
        stage += W_HEADING * heading_error_sq
        stage += obstacle_cost(px, py, hazard_points, obs_warn_weight, obs_safe_weight)
        stage += W_SPEED * (v - V_REF) ** 2
        stage += W_OMEGA * (omega - OMEGA_REF) ** 2
        total += scale * stage
        if k < nc_horizon:
            dv = controls[k, 0] - previous_v
            domega = controls[k, 1] - previous_omega
            total += W_DELTA_V * dv * dv + W_DELTA_OMEGA * domega * domega
            total += right_bypass_cost(omega, right_bypass_active)
            previous_v, previous_omega = controls[k]

    terminal_idx = pred_indices[-1]
    terminal_px, terminal_py = prediction[-1][0], prediction[-1][1]
    total += W_TERMINAL * (
        (terminal_px - ref_x[terminal_idx]) ** 2
        + (terminal_py - ref_y[terminal_idx]) ** 2
    )
    return float(total)


def make_bounds(nc_horizon):
    return [(V_MIN, V_MAX), (OMEGA_MIN, OMEGA_MAX)] * nc_horizon


def make_increment_constraints(last_control, nc_horizon):
    def increment_ineq(seq):
        values = []
        for i in range(nc_horizon):
            prev_v = last_control[0] if i == 0 else seq[2 * (i - 1)]
            prev_omega = last_control[1] if i == 0 else seq[2 * (i - 1) + 1]
            dv = seq[2 * i] - prev_v
            domega = seq[2 * i + 1] - prev_omega
            values.extend(
                [DV_MAX - dv, DV_MAX + dv, DOMEGA_MAX - domega, DOMEGA_MAX + domega]
            )
        return np.asarray(values, dtype=float)

    return [{"type": "ineq", "fun": increment_ineq}]


def project_control_sequence_to_feasible(seq, last_control, nc_horizon):
    """Sequentially enforce command bounds and per-step increment limits."""
    array = np.asarray(seq, dtype=float).reshape(-1)
    if len(array) != 2 * nc_horizon or not np.all(np.isfinite(array)):
        array = np.tile(np.asarray(last_control, dtype=float), nc_horizon)
    controls = array.reshape(nc_horizon, 2).copy()
    previous = np.asarray(last_control, dtype=float).copy()
    for index in range(nc_horizon):
        v = float(np.clip(controls[index, 0], V_MIN, V_MAX))
        omega = float(np.clip(controls[index, 1], OMEGA_MIN, OMEGA_MAX))
        v = float(np.clip(v, previous[0] - DV_MAX, previous[0] + DV_MAX))
        omega = float(
            np.clip(omega, previous[1] - DOMEGA_MAX, previous[1] + DOMEGA_MAX)
        )
        controls[index] = [np.clip(v, V_MIN, V_MAX), np.clip(omega, OMEGA_MIN, OMEGA_MAX)]
        previous = controls[index]
    return controls.reshape(-1)


def control_sequence_is_feasible(seq, last_control, nc_horizon, tolerance=None):
    tolerance = SOLVER_FEASIBILITY_TOL if tolerance is None else float(tolerance)
    array = np.asarray(seq, dtype=float).reshape(-1)
    if len(array) != 2 * nc_horizon or not np.all(np.isfinite(array)):
        return False
    controls = array.reshape(nc_horizon, 2)
    if np.any(controls[:, 0] < V_MIN - tolerance) or np.any(controls[:, 0] > V_MAX + tolerance):
        return False
    if np.any(controls[:, 1] < OMEGA_MIN - tolerance) or np.any(controls[:, 1] > OMEGA_MAX + tolerance):
        return False
    previous = np.asarray(last_control, dtype=float)
    for control in controls:
        if abs(control[0] - previous[0]) > DV_MAX + tolerance:
            return False
        if abs(control[1] - previous[1]) > DOMEGA_MAX + tolerance:
            return False
        previous = control
    return True


def nominal_control_sequence(nc_horizon):
    return np.tile(np.array([V_REF, OMEGA_REF], dtype=float), nc_horizon)


def adapt_warm_start(old_start, old_nc, new_nc, last_control):
    if old_start is None or len(old_start) != 2 * old_nc or not np.all(np.isfinite(old_start)):
        return project_control_sequence_to_feasible(
            np.tile(last_control, new_nc), last_control, new_nc
        )
    controls = np.asarray(old_start, dtype=float).reshape(old_nc, 2)
    if new_nc <= old_nc:
        adapted = controls[:new_nc]
    else:
        adapted = np.vstack([controls, np.tile(controls[-1], (new_nc - old_nc, 1))])
    return project_control_sequence_to_feasible(adapted.reshape(-1), last_control, new_nc)


def reset_warm_start_for_grid(last_control, nc_horizon):
    nominal = nominal_control_sequence(nc_horizon)
    reset = (
        GRID_RESET_LAST_CONTROL_BLEND * np.tile(last_control, nc_horizon)
        + GRID_RESET_NOMINAL_BLEND * nominal
    )
    return project_control_sequence_to_feasible(reset, last_control, nc_horizon)


def cbf_safety_filter(
    x,
    y,
    theta,
    v_raw,
    omega_raw,
    hazard_points,
    last_control,
    cbf_config,
):
    """CBF-like final output filter using only sensor-derived hazard points."""
    v_safe = float(v_raw)
    omega_safe = float(omega_raw)
    active = False
    position = np.array([x, y], dtype=float)
    heading = np.array([np.cos(theta), np.sin(theta)], dtype=float)
    critical_distance = np.inf

    for item in hazard_points or []:
        point = np.asarray(item["point"], dtype=float)
        relative = position - point
        distance = max(float(np.linalg.norm(relative)), 1e-9)
        h_value = distance * distance - cbf_config["safe_dist"] ** 2
        approach = float(np.dot(relative, heading))
        safety_value = 2.0 * approach * v_safe + cbf_config["gamma"] * h_value
        critical_distance = min(critical_distance, distance)
        if distance < cbf_config["trigger_margin"] and safety_value < 0.0:
            active = True
            if approach < 0.0:
                v_bound = cbf_config["gamma"] * max(h_value, 0.0) / max(-2.0 * approach, 1e-9)
                v_safe = min(v_safe, max(V_MIN, v_bound))
            if h_value < 0.0:
                v_safe = min(v_safe, max(V_MIN, cbf_config["min_v_scale"] * v_raw))

    if active:
        span = max(cbf_config["trigger_margin"] - cbf_config["safe_dist"], 1e-9)
        strength = float(
            np.clip((cbf_config["trigger_margin"] - critical_distance) / span, 0.0, 1.0)
        )
        omega_safe -= cbf_config["tangential_omega"] * strength

    projected = project_control_sequence_to_feasible(
        np.array([v_safe, omega_safe]), last_control, 1
    )
    return float(projected[0]), float(projected[1]), bool(active)


class ProgressiveMPC:
    def __init__(self, version):
        self.version = version
        self.last_ref_idx = INITIAL_REF_INDEX
        self.last_control = np.array([V_REF, 0.0], dtype=float)
        self.current_np = int(version["fixed_np"])
        self.current_nc = int(version["fixed_nc"])
        self.warm_start = np.tile(self.last_control, self.current_nc)
        self.hazard_points = []
        self.prev_d_min = None
        self.prev_risk = None
        self.risk = 0.0
        self.raw_risk_level = "LOW"
        self.control_risk_state = StableRiskLevel()
        self.control_risk_level = "LOW"
        self.d_min = SENSOR_MAX_RANGE
        self.closing_rate = 0.0
        self.right_bypass = FixedRightBypass()
        self.previous_pattern_id = None
        self.prediction_grid_switch_count = 0
        self.solver_failure_count = 0
        self.solver_fallback_count = 0

    def control(self, x, y, theta, reference, sensors):
        ref_x, ref_y, ref_theta, ref_s = reference
        self.last_ref_idx = nearest_forward_index(x, y, ref_x, ref_y, self.last_ref_idx)
        self.hazard_points = build_virtual_hazard_points(
            x, y, theta, sensors, self.hazard_points
        )

        raw_risk, self.raw_risk_level, self.d_min, self.closing_rate = compute_fuzzy_risk(
            sensors, self.prev_d_min, DT_CONTROL
        )
        self.risk = raw_risk if self.prev_risk is None else (
            RISK_FILTER_ALPHA * self.prev_risk + (1.0 - RISK_FILTER_ALPHA) * raw_risk
        )
        self.risk = clamp01(self.risk)
        self.control_risk_level, risk_level_switched = self.control_risk_state.update(
            self.risk
        )
        self.prev_risk = self.risk
        self.prev_d_min = self.d_min

        avoidance_active = self.d_min < POINT_WARNING_DIST
        bypass_active = self.right_bypass.update(sensors)
        np_horizon, nc_horizon = select_horizon(
            self.version, self.control_risk_level
        )
        prediction_dts = build_prediction_dts(
            self.version, np_horizon, self.control_risk_level
        )
        pattern_id = prediction_pattern_id(
            self.version, prediction_dts, self.control_risk_level
        )
        grid_switched = (
            self.previous_pattern_id is not None
            and pattern_id != self.previous_pattern_id
        )
        warm_start_grid_reset = False
        if self.version["nonuniform_time_horizon"] and grid_switched:
            self.warm_start = reset_warm_start_for_grid(self.last_control, nc_horizon)
            self.prediction_grid_switch_count += 1
            warm_start_grid_reset = True
        else:
            self.warm_start = adapt_warm_start(
                self.warm_start, self.current_nc, nc_horizon, self.last_control
            )
        self.previous_pattern_id = pattern_id
        self.current_np, self.current_nc = np_horizon, nc_horizon

        obs_warn_weight, obs_safe_weight = dynamic_obstacle_weights(
            self.version, self.risk
        )
        pred_indices = build_prediction_indices(
            ref_s, self.last_ref_idx, prediction_dts
        )
        nominal = nominal_control_sequence(nc_horizon)
        initial_guess = project_control_sequence_to_feasible(
            WARM_START_INITIAL_BLEND * self.warm_start
            + WARM_START_NOMINAL_BLEND * nominal,
            self.last_control,
            nc_horizon,
        )
        cost_args = (
            (x, y, theta),
            ref_x,
            ref_y,
            ref_theta,
            pred_indices,
            self.hazard_points,
            self.last_control,
            np_horizon,
            nc_horizon,
            prediction_dts,
            obs_warn_weight,
            obs_safe_weight,
            bypass_active,
        )
        solver_start = time.perf_counter()
        result = minimize(
            mpc_cost,
            initial_guess,
            args=cost_args,
            method=SOLVER_METHOD,
            bounds=make_bounds(nc_horizon),
            constraints=make_increment_constraints(self.last_control, nc_horizon),
            options={"maxiter": SOLVER_MAXITER, "ftol": SOLVER_FTOL, "disp": False},
        )
        optimizer_solve_time = time.perf_counter() - solver_start
        result_x_valid = (
            result.x is not None
            and np.all(np.isfinite(result.x))
            and control_sequence_is_feasible(result.x, self.last_control, nc_horizon)
        )
        solver_success = bool(
            result.success
            and result_x_valid
            and result.fun is not None
            and np.isfinite(result.fun)
        )
        solver_fallback_used = not solver_success
        if solver_success:
            solution = project_control_sequence_to_feasible(
                result.x, self.last_control, nc_horizon
            )
        else:
            self.solver_failure_count += 1
            self.solver_fallback_count += 1
            if (
                self.warm_start is not None
                and len(self.warm_start) == 2 * nc_horizon
                and np.all(np.isfinite(self.warm_start))
            ):
                fallback = self.warm_start
            else:
                fallback = np.tile(self.last_control, nc_horizon)
            # Failed result.x is deliberately ignored.
            solution = project_control_sequence_to_feasible(
                fallback, self.last_control, nc_horizon
            )

        controls = solution.reshape(nc_horizon, 2)
        v_raw, omega_raw = map(float, controls[0])
        fallback_v = v_raw if solver_fallback_used else np.nan
        fallback_omega = omega_raw if solver_fallback_used else np.nan
        if self.version["cbf"]:
            v_cmd, omega_cmd, cbf_active = cbf_safety_filter(
                x,
                y,
                theta,
                v_raw,
                omega_raw,
                self.hazard_points,
                self.last_control,
                self.version["cbf_config"],
            )
        else:
            v_cmd, omega_cmd, cbf_active = v_raw, omega_raw, False

        self.last_control = np.array([v_cmd, omega_cmd], dtype=float)
        shifted = np.vstack([controls[1:], controls[-1]]).reshape(-1)
        self.warm_start = project_control_sequence_to_feasible(
            WARM_START_SHIFT_BLEND * shifted
            + WARM_START_POST_NOMINAL_BLEND * nominal,
            self.last_control,
            nc_horizon,
        )
        return {
            "v": v_cmd,
            "omega": omega_cmd,
            "v_raw": v_raw,
            "omega_raw": omega_raw,
            "cbf_active": cbf_active,
            "v_correction": v_cmd - v_raw,
            "omega_correction": omega_cmd - omega_raw,
            "risk": self.risk,
            "raw_risk_level": self.raw_risk_level,
            "control_risk_level": self.control_risk_level,
            "risk_level_switched": risk_level_switched,
            "risk_level_switch_count": self.control_risk_state.switch_count,
            "d_min_sensor": self.d_min,
            "closing_rate": self.closing_rate,
            "obs_warn_weight": obs_warn_weight,
            "obs_safe_weight": obs_safe_weight,
            "np": np_horizon,
            "nc": nc_horizon,
            "prediction_total_time": float(np.sum(prediction_dts)),
            "prediction_dt_pattern_id": pattern_id,
            "prediction_grid_switched": grid_switched,
            "prediction_grid_switch_count": self.prediction_grid_switch_count,
            "warm_start_grid_reset": warm_start_grid_reset,
            "nonuniform_used": bool(not np.allclose(prediction_dts, prediction_dts[0])),
            "avoidance_active": avoidance_active,
            "right_bypass_active": bypass_active,
            "solver_success": solver_success,
            "solver_fallback_used": solver_fallback_used,
            "solver_status": int(getattr(result, "status", -1)),
            "solver_message": str(getattr(result, "message", "")),
            "fallback_v": fallback_v,
            "fallback_omega": fallback_omega,
            "failed_result_executed": False,
            "optimizer_solve_time": optimizer_solve_time,
        }
