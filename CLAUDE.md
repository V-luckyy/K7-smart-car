# K7 项目入口与当前约定

更新：2026-10-25。此文件只保留当前事实、决策、入口和待验证事项；操作细节见 docs，不在此累计工作日志。

## 1. 硬件与边界

- KICKPI K7 V2.0 / RK3576，目标系统 Ubuntu 24.04 + ROS 2 Jazzy。
- WHEELTEC C50X / C50C，STM32F407VET6，Keil `Diff_Car`，两轮差速；名义轮距 0.329 m、轮径 0.125 m，实车需核对。
- 每车本地 LRCP 2MV USB 双目，左右并排 MJPEG 3840×1080；标定与安装外参按车辆确认，不把示例配置视为测量结果。
- RK3576 经 CH9102F USB 串口连接 STM32 USART3，115200 / 8N1；默认别名 `/dev/k7_controller`、`/dev/k7_camera`，需核对 udev。
- K7 与底盘独立供电；底盘保留原有低压、禁能等保护，电池规格及保护行为仍需实测。
- 红外退役。已删除 GY53 驱动/显示页、STM32 APF/Stanley 模块及其调试帧；基础反馈保留，厂家共享驱动与保护逻辑保留。

## 2. 当前架构

`USB 双目 -> stereo_obstacles -> perception/scan`

`任务 Action / 遥控 -> task_manager -> controller_node -> command_guard -> serial_bridge -> STM32`

- ROS 负责感知、任务、APF + Stanley、编队和实验记录。STM32 只做速度执行、差速限幅、编码器/IMU反馈与独立命令超时停车。
- 7 个自有活动包：`k7_msgs`、`k7_bringup`、`k7_camera`、`k7_control`、`k7_tasks`、`k7_experiment`、`k7_description`。模型包不默认启动；外部相机驱动为 `usb_cam`。
- `robot.launch.py` 是统一车辆启动入口，profile 为 `manual / trajectory / formation`；启动后默认 IDLE，不自动行驶。
- 可选 `test_mode:=no_perception` 用于 `trajectory` 或 `formation`，跳过相机/双目节点，使用 Stanley-only，并在 guard 强制限速 0.05 m/s；默认 `none`。无任何视觉避障/近障保护，只允许封闭空旷场地测试，仍需现场急停。
- 长任务用 Action，连续量用 topic，授权/停车/记录开关用 service。`command_guard` 是活动链路唯一 `cmd_vel` 发布者。
- 一次只运行一个控制算法；正常模式目前仅 `apf_stanley`。无感知测试模式切换到 Stanley-only。旧 MPC、follower、APF recorder 包及旧串口/相机节点、旧 launch/脚本、红外消息已删除，工作区只有上述 7 包和一个车辆启动入口；历史实现查 Git。
- 当前仅支持前进，禁止倒车和原地旋转。未知/失效双目不当作自由空间；软件 hold 不是硬件急停。
- 编队只交换紧凑 `FleetState`，每车独立避障；必须确认各车 odom 到共同 fleet 坐标的初始变换并同步时钟。未实现全局定位、漂移修正或自动重定位。

## 3. 配置与日志

- 运行参数统一入口：`ROS2_src/K7_ros2/src/k7_bringup/config/robot.yaml`；launch 用 `config:=绝对路径` 选择车辆配置。
- 正常模式双目标定：`k7_camera/config/left.yaml`、`right.yaml` 或配置中的绝对路径；`calibration_confirmed` 默认 false，确认前不授权运动。无感知测试模式不启动相机节点，不提供避障保护。
- STM32 固定安全上限：`STM32F407VET6_src/USER/k7_config.h`。不能只提高 ROS 参数来突破固件限幅。
- 自有 Python 节点和串口桥参数启动时只读；更改配置须停车后重启 launch，禁止实验中改文件冒充当前生效配置。
- 记录器随车辆 launch 启动但默认不写盘；唯一启动开关为 `record:=true`，运行中也可用记录服务启停。变量、话题、采样率由配置选择。配置快照启动时缓存并核对 launch 哈希，防止后改文件冒充生效参数；`logs/README.md` 解释 manifest、CSV、bag，生成数据不入 Git。

## 4. 当前进度和验证边界

- ROS 接口、任务/权限状态机、双目扇区、APF/Stanley、无感知低速测试分支、串口桥、记录器及 STM32 ROS-only 修改已写入当前工作区。
- 当前 Windows 未发现 ROS 2 Jazzy / colcon / Keil 构建环境，不能宣称已编译、已刷写或实车生效。
- 纯 Python 回归覆盖差速限幅、Stanley、未知深度、编队初始对齐、授权撤销、任务取消和静止确认；结构检查覆盖 7 包/唯一 launch、安装输入、接口导出、Keil 引用与文档链接。运行：`python -m unittest discover -s tests -v`。
- 2026-10-25 本机 34 项回归通过；Python 编译、活动包 XML/配置 YAML 及文档链接检查通过。回归包含无感知测试模式的 Stanley 控制、scan 绕过边界、launch 配置和单一记录开关检查。架构图 2026-10-24 通过 Archify 检查并查看两种尺寸的明暗截图，主链路未改变。均不替代目标板/固件构建和实车测试。
- 待验收：K7 全量构建、Keil Diff_Car 构建、双目标定/遮挡/纹理不足测试、断链停车、低速单车跟踪、多车对齐和失联测试、CPU/延迟/带宽实测。
- SGBM 是主要算力/图像传输负担；配置频率不是性能测量结果。MPC 接入、动态规划、GUI 是扩展点，不是当前完成项。

## 5. 文档导航

| 内容 | 入口 |
|---|---|
| 当前文档总入口 | [文档目录](docs/README.md) |
| 包、节点、接口、负载、扩展约束 | [架构手册](docs/K7_开发架构方案.md) |
| 安装、标定、STM32、首次验收 | [上手指南](docs/K7_上手操作指南.md) |
| 启动、路径、遥控、停车、日志、排错 | [命令速查](docs/ROS2_命令速查.md) |
| 多车坐标、启动顺序、网络与风险 | [编队说明](docs/多机编队_LeaderFollower_说明.md) |
| 实验记录文件 | [日志说明](logs/README.md) |
| 当前工作区重构示意，非固定 commit 证据图 | [架构图](docs/k7-architecture.html) |

## 6. 目录与工作规则

```text
ROS2_src/K7_ros2/         自有 colcon 工作区、setup_env.sh
STM32F407VET6_src/        固件；USER/WHEELTEC.uvprojx
docs/                    当前手册与架构图
logs/                    实验输出（与 colcon log/ 区分）
tests/                   无 ROS 依赖的回归与静态检查
work_logs/               历史开发记录，不作为当前运行说明
stm32_data/              底盘硬件/历史算法参考
sensor_data/             退役红外参考资料
K7/、rk3576_data/         板卡原理图、规格和开发工具
ROS2_src/wheeltec_ros2/   第三方参考，不修改
```

工作前完整阅读本文；代码遵循 karpathy-guidelines 和 ponytail，保留用户已有修改。更新事实只修改对应条目，不追加对话流水账。目录变动同步 README、AGENTS 和 docs 引用；大文件、编译产物和实验数据不得误提交。提交格式：`<类型>: <简述>`。没有执行的构建、测量和上车验证必须明确标注。
