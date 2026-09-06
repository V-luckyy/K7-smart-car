"""MPC 版本注册表：V1 基线 + V5 全自适应（动态权重/视界/非均匀时域 + CBF 安全滤波）。

实车选定版本（2026-08-30）：V5。V1 保留，用于回退对比
（`ros2 launch k7_mpc k7_mpc.launch.py version:=V1`）。

相对仿真原文件（five_version_progressive_r04_with_pid/{V1_baseline,V5_cbf}/version_config.py）
的唯一改动：绝对导入 `from common import ...` 改为相对导入，适配 ROS2 ament_python
包结构；算法参数逐字未改。
"""

from . import common_config as common


V1_VERSION = {
    "key": "V1",
    "name": "Baseline MPC",
    "modules": ["continuous SLSQP MPC", "filtered hazard point", "fixed RIGHT bypass"],
    "dynamic_weight": False,
    "dynamic_horizon": False,
    "cbf": False,
    "nonuniform_time_horizon": False,
    "fixed_np": common.BASE_NP,
    "fixed_nc": common.BASE_NC,
}

V5_VERSION = {
    "key": "V5",
    "name": "Full Adaptive MPC + CBF-like Safety Filter",
    "modules": ["V4", "CBF-like safety filter"],
    "dynamic_weight": True,
    "dynamic_horizon": True,
    "cbf": True,
    "nonuniform_time_horizon": True,
    "fixed_np": common.BASE_NP,
    "fixed_nc": common.BASE_NC,
    "dynamic_weight_map": {
        "warn_base": 0.50,
        "warn_gain": 1.00,
        "safe_base": 45.0,
        "safe_gain": 30.0,
    },
    "horizon_map": {
        "LOW": (22, 4),
        "MEDIUM": (20, 5),
        "HIGH": (18, 6),
    },
    "prediction_dt_multipliers": {
        "LOW": [1] * 22,
        "MEDIUM": [1] * 14 + [2] * 6,
        "HIGH": [1] * 9 + [2] * 9,
    },
    "cbf_config": {
        # r04 sweep winner local_3x3_04；仿真复跑两次轨迹与核心指标一致。
        "safe_dist": 0.68,
        "trigger_margin": 1.08,
        "gamma": 1.50,
        "min_v_scale": 0.35,
        "tangential_omega": 0.32,
    },
}

# 注意：V5 HIGH 风险 horizon_map 用 NC=6（12 变量），K7 上 SLSQP 曾测 110~357ms、
# 超 70ms 控制预算（这也是当初把 BASE_NC 从 6 降到 4 的原因）。若上车后 HIGH 档
# 求解超时，可将 "HIGH": (22, 6) 改为 (22, 4) 收敛回 NC=4（一行改动，不影响 CBF 兜底）。

VERSIONS = {"V1": V1_VERSION, "V5": V5_VERSION}

# 默认实车版本（mpc_node 参数 version 的缺省值）
DEFAULT_VERSION = "V5"
