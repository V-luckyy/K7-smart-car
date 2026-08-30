"""V1: fixed-weight, fixed-horizon, uniform-step baseline MPC.

实车选定版本（2026-07-26）：先用 V1 跑通实车闭环，稳定后再升级到 V5(CBF)。
相对仿真原文件的唯一改动：绝对导入 `from common import ...` 改为相对导入，
适配 ROS2 ament_python 包结构。
"""

from . import common_config as common


VERSION = {
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
