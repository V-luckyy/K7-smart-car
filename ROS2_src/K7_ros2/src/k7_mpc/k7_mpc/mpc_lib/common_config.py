"""Single source of truth for parameters shared by all five versions.

The simulation truth values in this module are imported only by the sensor,
evaluation, and plotting layers. The MPC objective receives hazard points built
from three ray distances and never receives the true obstacle center or radius.

实车移植说明（2026-07-26）：算法逻辑未动，仅按架构方案 8.4 把下列参数改为
实车标定值（均以注释标出原仿真值）；OBSTACLE_* 等仿真真值在实车上不会被使用。
"""

from __future__ import annotations

import numpy as np


# Simulation timing and termination.
DT_CONTROL = 0.07
INTEGRATION_SUBSTEPS = 1
MAX_STEPS = 2600
FINISH_INDEX_MARGIN = 18

# Figure-eight reference path x=A*sin(t), y=A*sin(t)*cos(t).
# 仿真原值 4.0；实车路径幅度由 mpc_config.PATH_A 覆盖（generate_eight_path 支持传参）。
PATH_A = 4.0
PATH_POINTS = 1400
PATH_CYCLES = 1
REFERENCE_SEARCH_WINDOW = 90
INITIAL_REF_INDEX = 0

# Simulation truth. These values are forbidden from the MPC cost.
R_ROBOT = 0.25  # 实车标定 TODO：按 C50X 底盘实测量取（仿真值 0.30）
SIM_OBS_RADIUS = 0.50
OBSTACLE_T = np.pi / 4.0
OBSTACLE_CENTER = np.array(
    [
        PATH_A * np.sin(OBSTACLE_T),
        PATH_A * np.sin(OBSTACLE_T) * np.cos(OBSTACLE_T),
    ],
    dtype=float,
)
COLLISION_CENTER_DIST = R_ROBOT + SIM_OBS_RADIUS
SAFE_CENTER_DIST = 1.10

# Three ideal ray sensors: front, left 45 degrees, right 45 degrees.
SENSOR_MAX_RANGE = 1.2  # 实车标定 TODO：按红外 datasheet 量程填写（仿真值 3.0）
SENSOR_ANGLE_OFFSETS = {
    "front": 0.0,
    "left45": np.pi / 4.0,
    "right45": -np.pi / 4.0,
}

# Unknown-size obstacle representation. These are distances from the robot
# center to an observed obstacle-boundary hazard point, not to a known center.
POINT_COLLISION_DIST = R_ROBOT
POINT_SAFE_DIST = R_ROBOT + 0.35
POINT_WARNING_DIST = R_ROBOT + 0.70
VIRTUAL_OBS_HOLD_STEPS = 4
HAZARD_FILTER_ALPHA = 0.65

# Baseline horizons. V1 and V2 use these fixed values. V3-V5 may select their
# NP/NC from their version configuration, but share the same MPC implementation.
BASE_NP = 40
BASE_NC = 6

# Vehicle command bounds and reference values.
V_REF = 0.22
V_PATH_PROGRESS = 0.18
V_MIN = 0.08
V_MAX = 0.30  # 实车保守限幅（仿真值 0.42），验证稳定后逐步放开
OMEGA_REF = 0.0
OMEGA_MIN = -0.8  # 实车保守限幅（仿真值 ±1.2）
OMEGA_MAX = 0.8
DV_MAX = 0.08
DOMEGA_MAX = 0.25

# Fixed tracking and control weights inherited from the selected current V1.
W_TRACK = 8.0
W_HEADING = 1.0
W_OBS_WARN = 0.50
W_OBS_SAFE = 45.0
W_SPEED = 25.0
W_OMEGA = 0.35
W_DELTA_V = 3.0
W_DELTA_OMEGA = 3.0
W_TERMINAL = 45.0

# Fixed SLSQP and warm-start settings.
SOLVER_METHOD = "SLSQP"
SOLVER_MAXITER = 100
SOLVER_FTOL = 1e-4
WARM_START_INITIAL_BLEND = 0.75
WARM_START_NOMINAL_BLEND = 0.25
WARM_START_SHIFT_BLEND = 0.80
WARM_START_POST_NOMINAL_BLEND = 0.20

# Shared fuzzy-risk interpretation. V1 records this risk but does not feed it
# into control. V2-V5 use the same rules and filter.
RISK_FILTER_ALPHA = 0.70
RISK_LOW_THRESHOLD = 0.30
RISK_HIGH_THRESHOLD = 0.65
RISK_MEDIUM_ENTER = 0.35
RISK_MEDIUM_EXIT = 0.25
RISK_HIGH_ENTER = 0.70
RISK_HIGH_EXIT = 0.55
RISK_LEVEL_HOLD_STEPS = 6

# Shared fixed-right, inward bypass condition. Positive omega is a left turn,
# so a right bypass is encouraged by penalizing positive omega only while the
# obstacle is active. It is a soft MPC term, not a rule-based replacement.
SIDE_ENTER_DIST = POINT_WARNING_DIST
SIDE_CLEAR_DIST = 1.25
SIDE_CLEAR_STEPS = 5
W_SIDE_LOCK = 50.0

# Shared reliable fallback and prediction-grid reset settings.
# r04 numerical-tolerance correction shared by V1-V5.  Every accepted sequence
# is still projected to the exact actuator and increment limits before use.
SOLVER_FEASIBILITY_TOL = 1e-5
GRID_RESET_LAST_CONTROL_BLEND = 0.70
GRID_RESET_NOMINAL_BLEND = 0.30

# Shared metric definitions.
ENERGY_OMEGA_WEIGHT = 0.5
SMOOTH_OMEGA_WEIGHT = 0.5
RECOVERY_TRACK_ERROR = 0.15
RECOVERY_STABLE_STEPS = 5
OMEGA_SIGN_EPS = 0.03

# Comparison/plotting constants.
SOLVE_TIME_REFERENCE = DT_CONTROL


# The audit uses this explicit list. Version files are not allowed to redefine
# any of these names; they may only declare version-specific module parameters.
COMMON_PARAMETER_NAMES = [
    "DT_CONTROL",
    "INTEGRATION_SUBSTEPS",
    "MAX_STEPS",
    "FINISH_INDEX_MARGIN",
    "PATH_A",
    "PATH_POINTS",
    "PATH_CYCLES",
    "REFERENCE_SEARCH_WINDOW",
    "INITIAL_REF_INDEX",
    "R_ROBOT",
    "SIM_OBS_RADIUS",
    "OBSTACLE_T",
    "OBSTACLE_CENTER",
    "COLLISION_CENTER_DIST",
    "SAFE_CENTER_DIST",
    "SENSOR_MAX_RANGE",
    "SENSOR_ANGLE_OFFSETS",
    "POINT_COLLISION_DIST",
    "POINT_SAFE_DIST",
    "POINT_WARNING_DIST",
    "VIRTUAL_OBS_HOLD_STEPS",
    "HAZARD_FILTER_ALPHA",
    "BASE_NP",
    "BASE_NC",
    "V_REF",
    "V_PATH_PROGRESS",
    "V_MIN",
    "V_MAX",
    "OMEGA_REF",
    "OMEGA_MIN",
    "OMEGA_MAX",
    "DV_MAX",
    "DOMEGA_MAX",
    "W_TRACK",
    "W_HEADING",
    "W_OBS_WARN",
    "W_OBS_SAFE",
    "W_SPEED",
    "W_OMEGA",
    "W_DELTA_V",
    "W_DELTA_OMEGA",
    "W_TERMINAL",
    "SOLVER_METHOD",
    "SOLVER_MAXITER",
    "SOLVER_FTOL",
    "WARM_START_INITIAL_BLEND",
    "WARM_START_NOMINAL_BLEND",
    "WARM_START_SHIFT_BLEND",
    "WARM_START_POST_NOMINAL_BLEND",
    "RISK_FILTER_ALPHA",
    "RISK_LOW_THRESHOLD",
    "RISK_HIGH_THRESHOLD",
    "RISK_MEDIUM_ENTER",
    "RISK_MEDIUM_EXIT",
    "RISK_HIGH_ENTER",
    "RISK_HIGH_EXIT",
    "RISK_LEVEL_HOLD_STEPS",
    "SIDE_ENTER_DIST",
    "SIDE_CLEAR_DIST",
    "SIDE_CLEAR_STEPS",
    "W_SIDE_LOCK",
    "SOLVER_FEASIBILITY_TOL",
    "GRID_RESET_LAST_CONTROL_BLEND",
    "GRID_RESET_NOMINAL_BLEND",
    "ENERGY_OMEGA_WEIGHT",
    "SMOOTH_OMEGA_WEIGHT",
    "RECOVERY_TRACK_ERROR",
    "RECOVERY_STABLE_STEPS",
    "OMEGA_SIGN_EPS",
]
