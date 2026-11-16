# K7 智能小车

RK3576 / Ubuntu 24.04 / ROS 2 Jazzy + STM32F407 两轮差速底盘。正常模式使用每车 USB 双目感知和 ROS APF + Stanley；可选无感知低速测试模式用于单车路径/多车编队功能验证。STM32 执行底盘控制。

**状态：7 包构建与无感知三车编队验证通过；Keil 构建/刷写、双目标定和落地循迹待验收。启动默认不运动。**

## 文档

| 要查什么 | 入口 |
|---|---|
| 当前事实、约定与索引 | [CLAUDE.md](CLAUDE.md) |
| 包、节点、话题、服务、Action、ESP-NOW、负载 | [架构手册](docs/K7_开发架构方案.md) |
| 整体数据流 | [架构图](docs/k7-architecture.html) |
| 安装、升级清理、标定、STM32、验收 | [上手指南](docs/K7_上手操作指南.md) |
| 启动、路径、遥控、停车、记录、排错 | [命令速查](docs/ROS2_命令速查.md) |
| 多车坐标、启动顺序、失联处理 | [编队说明](docs/多机编队_LeaderFollower_说明.md) |
| Windows 双板 ESP-NOW 收发测试 | [测试说明](ESP32_src/windows_test/README.md) |
| 实验记录格式、共用分析脚本 | [日志说明](logs/README.md) |
| 未完成任务与交付边界 | [开发任务清单](docs/开发任务清单.md) |
| 开发规范 | [AGENTS.md](AGENTS.md) |

## 目录

```text
ROS2_src/K7_ros2/src/  7 个自有包（msgs/bringup/camera/control/tasks/experiment/description）
STM32F407VET6_src/     Keil 工程，底盘执行与保护
ESP32_src/             ESP-NOW 固件与 Windows 双板测试
docs/                  当前手册与架构图
logs/                  实验数据（生成内容不提交）
tools/                 共用小工具（编队分析/绘图、odom 记录）
tests/                 无 ROS 依赖的回归与静态检查
work_logs/             历史开发记录
K7/、rk3576_data/、stm32_data/、sensor_data/  板卡、底盘与历史资料
```

## 最短启动

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch k7_bringup robot.launch.py namespace:=car1 profile:=trajectory
```

多车编队见[编队说明](docs/多机编队_LeaderFollower_说明.md)，ESP-NOW 一主两从见[架构手册](docs/K7_开发架构方案.md)第 8 节。功能测试可加 `test_mode:=no_perception`：跳过相机、Stanley-only、guard 限速；无避障，仅封闭空旷场地并保留现场急停。

## 协作

- 从 `main` 拉分支，完成一项后运行 `python -m unittest discover -s tests -v`，提交 PR 并附未验证项。
- 提交格式 `<docs|feat|fix|refactor|chore>: <简述>`；不要把固件产物、密钥或日志带进提交。
- 板端 `~/Code/K7_ros2` 只放源码与标定；先核对两端源码版本再同步，不要把构建产物复制回来。
- KICKPI 资料：[中文站](https://kickpi.cn) · [英文站](https://www.kickpi.com) · [K7 上手文档](https://doc.kickpi.cn/Products/Beginner-Guide/KICKPI-K7/)
