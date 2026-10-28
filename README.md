# K7 智能小车

RK3576 / Ubuntu 24.04 / ROS 2 Jazzy + STM32F407 两轮差速底盘。正常模式使用每车 USB 双目感知和 ROS APF + Stanley；可选无感知低速测试模式用于单车路径/多车编队功能验证。STM32 执行底盘控制。

**状态：K7 的 7 包构建和无感知架空链路验证通过；Keil 构建/刷写及落地循迹待验收。启动默认不运动，正常感知模式标定未确认时拒绝授权。**

## 从哪里开始

| 要做什么 | 文档 |
|---|---|
| 浏览当前文档目录 | [docs/README.md](docs/README.md) |
| 掌握当前约定和进度 | [CLAUDE.md](CLAUDE.md) |
| 查看整体数据流 | [交互架构图](docs/k7-architecture.html) |
| 查看包、节点、接口和扩展方式 | [架构手册](docs/K7_开发架构方案.md) |
| 部署与首次测试 | [上手指南](docs/K7_上手操作指南.md) |
| 找启动、任务、停车、记录命令 | [命令速查](docs/ROS2_命令速查.md) |
| 测试多车编队 | [编队说明](docs/多机编队_LeaderFollower_说明.md) |
| 配置实验记录 | [logs/README.md](logs/README.md) |
| 领取尚未完成的模块任务 | [开发任务清单](docs/开发任务清单.md) |

## 目录

```text
ROS2_src/K7_ros2/src/
  k7_msgs/         消息、服务和 Action
  k7_bringup/      launch、统一配置、串口桥、udev
  k7_camera/       双目感知及标定工具
  k7_control/      APF/Stanley 和 command_guard
  k7_tasks/        手动、路径和编队任务
  k7_experiment/   可选 CSV / rosbag 记录
  k7_description/ 模型资源，不默认启动
STM32F407VET6_src/  Keil 工程，底盘执行及保护
docs/              当前操作手册与架构图
logs/              实验数据（不提交生成内容）
tests/             回归检查
work_logs/         历史开发记录
```

硬件参考仍在 `K7/`、`rk3576_data/`、`stm32_data/`、`sensor_data/`；不需要每次读取。旧方案不再是运行说明。

## 最短启动路径

在 K7 的 `ROS2_src/K7_ros2` 工作区执行，先完成上手指南中的依赖、udev、固件和标定检查：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch k7_bringup robot.launch.py namespace:=car1 profile:=trajectory
```

单车路径或编队功能测试可在对应车辆启动命令增加 `test_mode:=no_perception`。该模式不启动相机处理、不做避障，软件限速 0.05 m/s；仅限封闭空旷场地，并保留现场急停。

这里只启动节点。确认传感器、静止及场地安全后再发送任务 Action；停止使用 `task/stop` 或 `control/hold`。不要直接向 `cmd_vel` 发命令绕过门控。

开发规范见 [AGENTS.md](AGENTS.md)。本机回归：`python -m unittest discover -s tests -v`。

## 板卡资料与协作

快速协作命令（从仓库根目录执行；占位符执行前替换）：

```bash
git clone https://github.com/V-luckyy/K7-smart-car.git
cd K7-smart-car
git pull --ff-only origin main
git switch -c 'feat/<task-name>'
git add '<changed-file-paths>'
git diff --cached --check
git diff --cached
git commit -m 'feat: 简述改动'
git push -u origin 'feat/<task-name>'
```

已有远端分支先 `git fetch origin`，再 `git switch --track origin/<branch>`；尖括号部分均为占位符。推送后提交 PR。

- KICKPI：[中文站](https://kickpi.cn)、[英文站](https://www.kickpi.com)、[K7 官方上手文档](https://doc.kickpi.cn/Products/Beginner-Guide/KICKPI-K7/)；本地原理图/工具在 `K7/`、`rk3576_data/`。烧录、调试串口、网线及 VS Code Remote-SSH 的步骤见[上手指南](docs/K7_上手操作指南.md)。
- 底盘固件/协议资料在 `stm32_data/`；第三方 wheeltec ROS 参考在 `ROS2_src/wheeltec_ros2/`（不作为当前运行代码）。
- 仓库主副本在本地工作区；板端 `~/Code/K7_ros2` 负责构建和实验。先核对两端源码版本，保存板端自改配置与标定，再同步；不要把旧 `src/` 合并进新源码或把编译产物复制回来。
- 协作流程：从 `main` 拉取最新代码，在独立分支完成清单中的一项，运行相关测试和目标机检查，提交 PR 并附实验记录/未验证项。提交格式为 `docs / feat / fix / refactor / chore: 简述`；不要用 `git add .` 把固件产物、密钥或日志带进提交。
