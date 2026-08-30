"""k7_mpc 节点级实车配置。

算法参数（权重 / 视界 / 限幅 / 传感器量程等）在 mpc_lib/common_config.py，
实车化改动已在该文件内以注释标出；本文件只管 ROS 接线与参考路径。
"""

# ---- 参考路径（硬编码，不接规划器）----
# 8 字形 x=A*sin(t), y=A*sin(t)*cos(t)，总跨度 2A 米。仿真 PATH_A=4.0（8m 跨度）。
PATH_A = 1.2

# ---- 话题 ----
ODOM_TOPIC = "/odom_combined"          # EKF 融合里程计（nav_msgs/Odometry）
CMD_TOPIC = "/cmd_vel_mpc"             # 输出给 twist_mux（geometry_msgs/Twist）
IR_TOPIC = "/ir_distances"             # 3 路红外单话题（k7_msgs/IrDistances）
IR_NAMES = ("front", "left45", "right45")  # 字段名与 IrDistances.msg 一致
