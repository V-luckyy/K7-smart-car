# K7(RK3576) ↔ STM32(C50X) 串口协议速查

> 链路：K7 USB Host → CH9102F → STM32 USART3（PC10/PC11），K7 端设备 `/dev/k7_controller`（udev 固定）。
> 参数：**115200 8N1**。
> 主机为 K7，但 **STM32 主动周期上行**（数据任务 20Hz 推），下行按需 + 看门狗兜底。
> 源码：固件侧 `STM32F407VET6_src/BALANCE/data_task.c`(上行组帧)、`uartx_callback.c/h`(下行解析)；ROS 侧 `k7_serial_node.cpp`(`Send_Cmd_Vel` / `Get_Sensor_Data_New`)。
> 本协议与 wheeltec 11/24 字节帧**逐字节一致**（ROS 端就是移植 wheeltec 串口节点）。

---

## 0. 通用约定

| 项 | 规则 |
|---|---|
| 字节序 | 所有 int16/数值 **大端**（高字节在前） |
| 数值缩放 | `传输值 = 物理量 × 1000`（int16）。线速度 0.5 m/s → 500；收端 `/1000` |
| BCC | 帧头起逐字节异或，**到 BCC 前一位为止**（不含 BCC、不含帧尾） |
| 一帧一命令 | 固定长度帧，靠**帧头字节 + 长度**区分类型；解析端字节级重同步 |
| 速度坐标 | 整车（底盘）速度，非原始编码器；ROS 帧为 x 前 y 左 z 上，yaw 逆时针正 |

---

## 1. 下行：K7 → STM32（11 字节，`0x7B … 0x7D`）

发送方 `k7_serial_node::Send_Cmd_Vel()`。**同一个 11B 模板，靠字节[1]区分命令。**

| 偏移 | 长度 | 内容 | 说明 |
|---|---|---|---|
| 0 | 1 | `0x7B` | 帧头 |
| 1 | 1 | 命令字节 | 见下表 |
| 2 | 1 | 预留 | 恒 0 |
| 3–4 | 2 | Vx | int16 大端 ×1000，前向线速度 m/s |
| 5–6 | 2 | Vy | 侧向线速度 m/s（差速车恒 0） |
| 7–8 | 2 | Vz | 角速度 rad/s |
| 9 | 1 | BCC | 前 9 字节（0–8）异或 |
| 10 | 1 | `0x7D` | 帧尾 |

**命令字节 [1] 语义（STM32 `uartx_callback.c`）：**

| [1] | 含义 |
|---|---|
| `0` | **普通 ROS 速度**：置 `_ROS_Control`(USART3) 控制模式、清 `AllowRecharge`，把 Vx/Vy/Vz 写入 `robot_control` 由 balance_task 驱动 ← **k7_serial_node 平时发的就是 0** |
| `1` / `2` | 自动回充：`AllowRecharge=1`，把 Vx/Vy/Vz 写入 `charger.Up_MoveX/Y/Z`（回充寻桩速度）；1/2 差异是是否进入 NavWalk/寻桩逻辑 |
| `3` | 红外对接速度设置：Vx/Vy/Vz → `charger.Dock_MoveX/Y/Z` |
| `0xFF` | IP 帧（[3..6]=4 字节 IP，STm32 记忆用；k7_serial_node 未用） |
| 其它 | 预留/忽略 |

> 注意：收到 `0` 命令帧会**切到 ROS/串口控制模式**。APF 独立固件里 `apf_task` 每 20ms 把 `ControlMode` 清回 0 抢控，所以会压过串口（这就是"APF 自跑 / ROS 控制两种模式二选一"的固件层根源）。

**k7_serial_node 何时下发：**
- 收到 `cmd_vel`（twist_mux 仲裁输出）即发一条 `0` 速度帧；
- 之后无新 cmd_vel，100ms 看门狗检查、**超时 500ms 后每 100ms 发一条全 0 速度帧**（K7 固件断链不停车，必须软件兜底）；
- `set_charge` 服务 / 回充相关服务按需发 `1/2` 命令帧；
- 参数 `enable_downlink=false`（记录调参模式）时**只收不发**。

---

## 2. 上行：STM32 → K7（数据任务 20Hz 一批发出）

`data_task` 每 50ms 依次发：24B 主帧 + 19B 测距帧(有红外硬件) + 8B 回充帧(有回充/红外硬件) + 27B APF 调试帧(APF 任务在跑时)。`k7_serial_node` 按 0x7B/0xFA/0x7C/0xFB 帧头分派。

### 2.1 主帧 24B `0x7B … 0x7D`（20Hz，一直有）

| 偏移 | 长度 | 内容 | 单位·收端换算 |
|---|---|---|---|
| 0 | 1 | `0x7B` | 帧头 |
| 1 | 1 | FlagStop/状态 | 0 正常；1=电机失能（此时 gyro Z 强制 0） |
| 2–3 | 2 | Vx | ×1000 大端 → `/1000` = m/s |
| 4–5 | 2 | Vy | 同上（全向车才有效） |
| 6–7 | 2 | Vz | ×1000 → rad/s |
| 8–9 | 2 | acc X | IMU 原始值（已做 ROS 坐标轴变换） |
| 10–11 | 2 | acc Y | |
| 12–13 | 2 | acc Z | |
| 14–15 | 2 | gyro X | IMU 原始值 |
| 16–17 | 2 | gyro Y | |
| 18–19 | 2 | gyro Z | |
| 20–21 | 2 | 电池电压 | mV 大端 → K7 换算 V |
| 22 | 1 | BCC | 前 22 字节（0–21）异或 |
| 23 | 1 | `0x7D` | 帧尾 |

→ K7 收后：速度做正运动学来的整车速度、积分成里程计；IMU 按加速度量程/陀螺量程缩放成 m/s²·rad/s；发布 `/odom`、`/imu/data_raw`、`/PowerVoltage`。

### 2.2 测距帧 19B `0xFA … 0xFC`

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 1 | `0xFA` |
| 1–2 | 2 | rangerA = 前方 0°（PE5）int16 大端 **mm** |
| 3–4 | 2 | rangerB = 左前 +45°（PE7） |
| 5–6 | 2 | rangerC = 右前 −45°（PE8） |
| 7–12 | 6 | rangerD/E/F（恒 5000，预留） |
| 13–16 | 4 | 预留 0 |
| 17 | 1 | BCC（0–16 异或） |
| 18 | 1 | `0xFC` |

→ K7 `/1000` = m，发布 `/ir_distances`（front/left45/right45）。无遮挡读数顶到测距模式上限（1.2m/2m），非 5m。

### 2.3 回充帧 8B `0x7C … 0x7F`

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 1 | `0x7C` |
| 1–2 | 2 | 充电电流 int16 大端 |
| 3 | 1 | RED：是否找到充电桩红外 |
| 4 | 1 | Charging：是否在充电 |
| 5 | 1 | AllowRecharge：固件是否允许自动回充 |
| 6 | 1 | BCC（0–5 异或） |
| 7 | 1 | `0x7F` |

→ 发布 `robot_charging_current` / `robot_red_flag` / `robot_charging_flag`。

### 2.4 APF/Stanley 调参调试帧 27B `0xFB … 0x7D`（2026-09-06 新增）

APF 任务 50Hz 采样进 `g_apf_debug`，data_task 20Hz 上行（调参专用，普通 ROS 控制固件不产生）。

| 偏移 | 长度 | 字段 | 内容 |
|---|---|---|---|
| 0 | 1 | `0xFB` | |
| 1–2 | 2 | x | 里程计 X ×1000 |
| 3–4 | 2 | y | |
| 5–6 | 2 | θ | rad×1000（未 wrap，长跑会溢出刻度，绘图侧 unwrap） |
| 7–8 | 2 | v_cmd | 下发线速度 |
| 9–10 | 2 | w_cmd | 下发角速度 = w_stanley + w_apf |
| 11–12 | 2 | w_stanley | 循迹转向分量 |
| 13–14 | 2 | w_apf | 避障转向分量 |
| 15–16 | 2 | v_act | 实际线速度 |
| 17–18 | 2 | w_act | 实际角速度 |
| 19–20 | 2 | d_front | 前方距离 **mm**（5000=无遮挡） |
| 21–22 | 2 | d_left | 左前 45° |
| 23–24 | 2 | d_right | 右前 −45° |
| 25 | 1 | BCC（0–24 异或） | |
| 26 | 1 | `0x7D` | |

→ 发布 `/apf_debug`（k7_msgs/ApfDebug），配合 `k7_apf_debug` 记录/绘图调参。

---

## 3. 帧 → ROS 话题映射（k7_serial_node）

| 上行帧 | ROS 话题 / 消息 |
|---|---|
| 0x7B 主帧 | `/odom`(nav_msgs/Odometry)、`/imu/data_raw`、`/PowerVoltage` |
| 0xFA 测距 | `/ir_distances`(k7_msgs/IrDistances) |
| 0x7C 回充 | `robot_charging_flag`/`robot_red_flag`/`robot_charging_current` |
| 0xFB 调试 | `/apf_debug`(k7_msgs/ApfDebug) |

| 下行 | 来源 |
|---|---|
| cmd_vel(0x7B,cmd=0) | twist_mux 输出 `cmd_vel` → 串口；看门狗零速帧同路径 |
| 回充命令(1/2) | `set_charge` 服务 / 回充服务 |
| 红外对接速度(3) | （回充相关服务，k7_bringup 有 Red_Vel_Sub 对应通道） |

> 相关文档：协议结构体定义 `stm32_data/协议参考/data_task.h`（上行）、`uartx_callback.h`（下行），CLAUDE.md 第 13 节。
