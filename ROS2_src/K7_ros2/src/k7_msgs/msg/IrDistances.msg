# 三路红外测距（GY-53 / VL53L0X ToF），单位：米
# 由 k7_serial_node 从 STM32 上行 0xFA 测距帧解析后发布到 /ir_distances。
# front / left45 / right45 与仿真传感器字典 {front, left45, right45} 及 k7_mpc 节点键名一致。
std_msgs/Header header

float32 front     # 正前方 0°（rangerA，STM32 PE5）
float32 left45    # 左前 +45°（rangerB，STM32 PE7）
float32 right45   # 右前 -45°（rangerC，STM32 PE8）
