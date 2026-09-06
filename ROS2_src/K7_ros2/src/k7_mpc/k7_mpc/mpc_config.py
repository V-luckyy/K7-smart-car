"""k7_mpc 节点级实车配置。

算法参数（权重 / 视界 / 限幅 / 传感器量程等）在 mpc_lib/common_config.py，
实车化改动已在该文件内以注释标出；本文件只管 ROS 接线与参考路径。
"""

# ---- 参考路径（硬编码，不接规划器）----
# PATH_TYPE: "eight" 8字形 / "circle" 圆形（测避障用，曲率恒定无尖弯）
PATH_TYPE = "circle"
PATH_A = 0.72              # 8 字幅度（PATH_TYPE="eight" 时用）
CIRCLE_RADIUS = 0.6        # 圆半径（PATH_TYPE="circle" 时用；直径 1.2m 适配 ~1.5m 空间）

# ---- 话题 ----
ODOM_TOPIC = "/odom_combined"          # EKF 融合里程计（nav_msgs/Odometry）
CMD_TOPIC = "/cmd_vel_mpc"             # 输出给 twist_mux（geometry_msgs/Twist）
IR_TOPIC = "/ir_distances"             # 3 路红外单话题（k7_msgs/IrDistances）
IR_NAMES = ("front", "left45", "right45")  # 字段名与 IrDistances.msg 一致
