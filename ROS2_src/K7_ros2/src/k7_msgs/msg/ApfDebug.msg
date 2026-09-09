# APF + Stanley 调参调试数据（STM32 上行 0xFB 帧，50Hz 采样 / 20Hz 上行）
# k7_serial_node 解析后发布到 /apf_debug。
# 作用：实车调参时把控制器内部量上行出来 —— 实际轨迹 vs 参考圆、指令/实际速度、避障分量。
# 单位：位置 m / 速度 m/s / 角速度 rad/s / 距离 m（无遮挡=5.0）
std_msgs/Header header

float64 x           # 里程计 X (m)
float64 y           # 里程计 Y (m)
float64 theta       # 偏航角 (rad)，未 wrap
float64 v_cmd       # 下发线速度 (m/s)
float64 w_cmd       # 下发角速度 (rad/s) = w_stanley + w_apf
float64 w_stanley   # 循迹转向分量 (rad/s)
float64 w_apf       # 避障转向分量 (rad/s)
float64 v_act       # 实际线速度 (m/s)（编码器）
float64 w_act       # 实际角速度 (rad/s)（编码器差分）
float32 d_front     # 前方 0°   测距 (m)，5.0=无遮挡
float32 d_left      # 左前 +45° 测距 (m)
float32 d_right     # 右前 -45° 测距 (m)
