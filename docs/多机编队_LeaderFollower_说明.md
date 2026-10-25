# K7 多车编队

正常模式下每车独立运行双目、控制器、guard 和串口桥。主车执行路径，从车订阅前车 `FleetState`，由 `k7_tasks/task_manager` 构造短轨迹并用自己的 APF + Stanley 跟踪。若只验证路径/编队功能，可对每辆车分别设置 `test_mode:=no_perception`：跳过相机/双目节点，改为 Stanley-only，最终速度限制为 0.05 m/s。该模式无避障和近障保护，只能在封闭空旷场地测试，现场急停必须可用。统一从 `k7_bringup/launch/robot.launch.py` 启动；当前实现限前进队列，未完成目标机/实车验收。

## 1. 坐标和网络前提

- 同一实验使用相同 `ROS_DOMAIN_ID`，每车不同 namespace；时钟必须同步，前车消息时效默认 0.5 s。namespace 只避免重名，不能对齐位置，也不是访问隔离。
- 每车 odom 原点在本次串口桥启动时重置。配置 `task_manager.fleet_x / fleet_y / fleet_yaw` 表示本车 odom 原点在共同 `fleet_frame` 中的位姿。
- 本地点变换为共同坐标：`p_fleet = R(fleet_yaw) * p_odom + (fleet_x, fleet_y)`。从车按本车变换的逆变换处理前车位置。
- 实测并确认初始变换后才设 `formation_alignment_confirmed: true`。前车和从车初始应同向排列，默认允许横偏 0.15 m、航向差 0.35 rad；并非任意摆放自动编队。
- 没有全局定位/漂移修正，不适合未经验证的长距离运行；任一车重启后旧坐标假设失效，必须全队停车重建对齐。

例：主车原点作为 fleet 原点，car2 在主车后方 0.8 m 且同向，则 car1 变换为 (0,0,0)，car2 为 (-0.8,0,0)。这是几何示例，不是本车实测值。队距沿前车轨迹计算，不是简单世界 x 差。

## 2. 启动

各车分别 source 系统和自己的工作区，在同一可信局域网运行：

```bash
export ROS_DOMAIN_ID=7
# car1，仅在主车执行；car1.yaml 已确认本车标定与 fleet 变换
ros2 launch k7_bringup robot.launch.py profile:=trajectory namespace:=car1 config:=$HOME/k7/car1.yaml publish_fleet:=true
```

```bash
export ROS_DOMAIN_ID=7
# car2，仅在从车执行
ros2 launch k7_bringup robot.launch.py profile:=formation namespace:=car2 config:=$HOME/k7/car2.yaml
```

仅做无感知低速验证时，上面每条车辆启动命令都加 `test_mode:=no_perception`；不需要图像标定，但仍须每车确认里程计、底盘禁能状态和编队坐标。car1 用 `profile:=trajectory publish_fleet:=true`，car2 用 `profile:=formation publish_fleet:=true`，car3 用 `profile:=formation` 并跟随 car2。测试模式下的 `FollowPath` 目标速度还会被限制到 0.05 m/s。

正常模式先检查各车 IDLE、scan 和 odom 有效；无感知测试模式检查 IDLE、odom 与底盘状态。在 car2 检查 `/car1/fleet/state` 确实可达。确认初始队列后，先给从车跟随任务，再给主车路径：

```bash
ros2 topic echo /car1/fleet/state --once
ros2 action send_goal /car2/task/follow_leader k7_msgs/action/FollowLeader "{leader_topic: /car1/fleet/state, gap: 0.8, max_speed: 0.10}" --feedback
```

主车路径命令见 [命令速查](ROS2_命令速查.md)。跟随 Action 持续到取消、停车或故障，不以主车暂时静止结束。需要 car3 时，让 car2 也以 `publish_fleet:=true` 启动；car3 跟随 `/car2/fleet/state`，并配置自己的初始变换。

## 3. 运行边界

| 场景 | 当前行为 |
|---|---|
| 主车绕障 | 从车记录前车实际位置序列，不广播主车完整计划 |
| 从车自身有障碍 | 本车 APF 处理，近障/未知区由 guard 停车 |
| 前车失联或状态故障 | 从车任务故障并撤销权限，不盲追最后速度 |
| 前车重启/session 改变 | 中止跟随，不拼接不同里程计会话 |
| 本车感知、里程计或候选命令过期 | guard 撤销权限；串口失联另有 STM32 独立超时 |
| 停车响应 | 只代表受理，须观察每车新鲜反馈和静止确认 |

没有全队一致停车服务，没有全局多车避碰调度或相互让行协议。前方双目不保证看到所有邻车和盲区；从车跟踪链路不等于可证明的队形安全。先单车、再双车、再增加数量，保留现场停机手段。

## 4. 通信、参数和日志

车间仅需 `FleetState`，不超过 10 Hz，携带共同坐标位姿、线/角速度、状态和 session UUID。图像、深度、scan、控制和反馈由本车消费。DDS 仍有发现流量；远程工具主动订阅图像/odom 会产生额外网络流量，当前不是强制的网络白名单隔离。

编队路径不超过 10 Hz，本地控制 20 Hz，最终命令 30 Hz，串口基础反馈 20 Hz；均为设计周期，非测量结果。SGBM 是主要计算负担，先关闭深度调试和不需要的记录，再测 CPU、延迟与失联时限。

配置统一见 `ROS2_src/K7_ros2/src/k7_bringup/config/robot.yaml`；参数更改需停车重启。每车独立 [记录实验](../logs/README.md)，需要前车状态时在本车记录配置 `topics` 中显式加入 `/car1/fleet/state`，不要改为录全部话题。

停车分别调用 `/car1/task/stop`、`/car2/task/stop`，逐车确认 IDLE 和静止；软件 hold 锁定也需逐车操作。网络中断时不能指望远端服务可达，固件超时和现场停机仍必须验证。
