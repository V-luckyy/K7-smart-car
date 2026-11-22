# K7 项目入口与当前约定

此文件只保留当前事实、决策和索引，供 agent 快速定位；操作细节见 `docs/`，不记录调试过程、历史阶段或流水日志。

## 1. 硬件与边界

- KICKPI K7 V2.0 / RK3576，Ubuntu 24.04 + ROS 2 Jazzy。
- WHEELTEC C50X/C50C，STM32F407VET6，Keil `Diff_Car`，两轮差速；名义轮距 0.329 m、轮径 0.125 m。
- 每车 LRCP 2MV USB 双目，3840×1080 并排 MJPEG；标定与外参按车辆确认，示例配置不算测量结果。
- RK3576 经 CH9102F USB 串口连 STM32 USART3（115200/8N1），默认别名 `/dev/k7_controller`、`/dev/k7_camera`。
- K7 与底盘独立供电；底盘保留原有低压/禁能保护。
- 红外已退役；STM32 只做速度执行、差速限幅、编码器/IMU 反馈和命令超时停车。

## 2. 当前架构

`USB 双目 -> stereo_obstacles -> perception/scan`

`任务 Action / 遥控 -> task_manager -> controller_node -> command_guard -> serial_bridge -> STM32`

- ROS 负责感知、任务、APF + Stanley、编队和记录；STM32 只执行速度与反馈。
- 7 个活动包：`k7_msgs`、`k7_bringup`、`k7_camera`、`k7_control`、`k7_tasks`、`k7_experiment`、`k7_description`；相机用外部 `usb_cam`。
- `robot.launch.py` 是唯一车辆入口，profile 为 `manual / trajectory / formation`，启动后 IDLE、不自动行驶。
- `test_mode:=no_perception` 跳过相机、Stanley-only，guard 限速 `test_speed_limit`；无避障，仅封闭空旷场地并保留现场急停。
- 长任务用 Action，连续量用 topic，授权/停车/记录用 service；`command_guard` 是活动链路唯一 `cmd_vel` 发布者。
- 一次只跑一种算法：正常模式仅 `apf_stanley`，无感知模式切 Stanley-only。
- 仅支持前进，禁止倒车和原地旋转；未知/失效双目不当作自由空间；软件 hold 不是硬件急停。
- 编队只交换紧凑 `FleetState`，每车独立避障；需确认 odom→fleet 初始变换并同步时钟，无全局定位/漂移修正/自动重定位。
- 可选 ESP-NOW 一主两从：`ESP32_src/espnow_bridge/` 固件 + `k7_tasks/espnow_bridge` USB/ROS 桥，96 字节协议、≤10 Hz；`espnow_role` + `vehicle_id` 启用，默认 off。详见架构手册第 8 节。

## 3. 配置与日志

- 参数统一入口 `k7_bringup/config/robot.yaml`；每车 `config:=绝对路径` 复制一份。自有节点参数启动只读，改配置须停车重启 launch。
- ESP-NOW 串口/信道/编号/时效在 `espnow_bridge` 段，`device` 用本车 by-id/by-path；固件 `board_config.h` 配角色/编号/信道/主车 STA MAC；Arduino-ESP32 3.3.0，K7 需 python3-serial。
- 双目标定 `k7_camera/config/{left,right}.yaml`；`calibration_confirmed` 默认 false，确认前不授权运动。
- STM32 固定安全上限 `STM32F407VET6_src/USER/k7_config.h`，ROS 参数不能突破。
- 参考路径在 `k7_tasks/ref_path.py`（`ros2 run k7_tasks ref_path`）：`SHAPES` 提供 `spiral`（原点 +x 起步、两圈、内径 1.0 m、外径 2.2 m、500 点）和 `figure8`（8 m × 4 m）；每车按 `formation_path` 跑自己的路径，初始/目标编队见 `INITIAL_FORMATION`/`TARGET_FORMATION`（纵队 0.5/1.0 m → 三角形，后车后退 0.9 m、横向 ±0.5 m），`target_speed` 0.10 m/s。
- 记录器随 launch 启动但默认不写盘；`record:=true` 或记录服务启停；格式见 `logs/README.md`，分析/记录脚本见 `tools/`。

## 4. 当前状态与验收边界

- 已实现：ROS 接口与任务/授权状态机、双目扇区、APF+Stanley、无感知分支、串口桥、记录器、STM32 ROS-only、ESP-NOW 固件与桥。
- 已验证（本机）：`tests/` 纯 Python 回归与静态检查；ESP-NOW 协议/桥接测试；Arduino-ESP32 3.3.0 下三份固件配置编译通过。
- 已验证（板端）：无感知三车编队（螺线 + 两跟随者）跑通，指标与图见 `logs/`。
- 待验收：Keil 构建/刷写、双目标定与遮挡测试、落地单车跟踪、多车对齐与失联、CPU/延迟/带宽基准。
- 限制：odom 是轮速积分不是真实位姿；无感知模式无避障；ESP-NOW 单向未加密，主车无法获知从车在线/故障。

## 5. 文档导航

| 内容 | 入口 |
|---|---|
| 包、节点、接口、负载、扩展约束 | [架构手册](docs/K7_开发架构方案.md) |
| 安装、标定、STM32、验收 | [上手指南](docs/K7_上手操作指南.md) |
| 启动、路径、遥控、停车、日志、排错 | [命令速查](docs/ROS2_命令速查.md) |
| 多车坐标、启动顺序、网络与风险 | [编队说明](docs/多机编队_LeaderFollower_说明.md) |
| 一主两从 ESP-NOW 固件、K7 桥与部署 | [架构手册](docs/K7_开发架构方案.md) 第 8 节 |
| Windows 双板 USB/无线收发验证 | [测试说明](ESP32_src/windows_test/README.md) |
| 实验记录格式与共用分析脚本 | [日志说明](logs/README.md) |
| 待办任务、负责边界、交付验收 | [开发任务清单](docs/开发任务清单.md) |
| 整体数据流图 | [架构图](docs/k7-architecture.html) |

## 6. 目录与工作规则

```text
ROS2_src/K7_ros2/         自有 colcon 工作区、setup_env.sh
STM32F407VET6_src/        固件；USER/WHEELTEC.uvprojx
ESP32_src/espnow_bridge/  ESP-NOW 固件、板级配置、96 字节线协议
ESP32_src/windows_test/   Windows 双板一键测试
docs/                    当前手册与架构图
logs/                    实验输出（与 colcon log/ 区分）
tools/                   共用小工具（编队分析/绘图、odom 记录）
tests/                   无 ROS 依赖的回归与静态检查
work_logs/               历史开发记录，不作为当前运行说明
stm32_data/              底盘硬件/历史算法参考
sensor_data/             退役红外参考资料
K7/、rk3576_data/         板卡原理图、规格和开发工具
ROS2_src/wheeltec_ros2/   第三方参考，不修改
```

工作前完整阅读本文；代码遵循 karpathy-guidelines 和 ponytail，保留用户已有修改。更新事实只修改对应条目，不追加对话流水账。目录变动同步 README、AGENTS 和 docs 引用；大文件、编译产物和实验数据不得误提交。提交格式：`<类型>: <简述>`。
