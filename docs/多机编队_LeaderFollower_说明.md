# K7 多机编队测试说明

> 适用：3 台 K7 + C50X，ROS 2 Jazzy，同一局域网与 ROS_DOMAIN_ID。

## 1. 当前结构

采用链式前车轨迹跟随：

`leader -> follower1 -> follower2`

- 主车 STM32 执行硬编码路径、Stanley 和 APF；K7 只发布 `/leader/odom`，禁止串口下行。
- 从车保存前车约 `target_gap + 4 m` 的轨迹点，目标是自身最近轨迹点前方 `lookahead` 处。
- 从车线速度由前车速度和路径弧长间距误差决定；角速度由前车轨迹曲率前馈、航向误差和横向误差组成。
- 从车三路 GY-53 产生本地 APF 角速度并降低前进速度，最后统一做差速轮速限幅。
- 每车只运行串口节点；编队控制不启动 EKF、Madgwick、TF 或 URDF 发布器。

这样 follower2 订阅 `/follower1/odom`，每级只保持 1.2 m。若两台从车都订阅 leader，第二台必须把目标间距设成约 2.4 m。

## 2. 已知前提

三套轮式里程计各自从零开始，没有共同地图，也没有测量车间相对位置的传感器。因此启动前必须：

1. 三车朝向一致，近似位于同一直线上。
2. leader 到 follower1、follower1 到 follower2 均按 `target_gap` 摆放。
3. 所有节点在车辆摆好且静止后启动；运行中不要单独重启某一台跟随节点。

该方案能沿前车实际绕障轨迹行驶，但编码器累计漂移仍会影响长距离编队。需要长期绝对队形精度时，必须增加 UWB、视觉标记或统一定位，本实现不伪造这种能力。

## 3. 固件模式

- leader：烧写已启用 `APF_task` 的自主运行固件。
- follower1/2：烧写 ROS 控制固件，必须禁用 `USER/main.c` 中的 `CreateTaskChecked(APF_task, ...)`。从车 APF 在 K7 跟随节点中运行，STM32 只执行串口下发的同时线速度/角速度。

主车 `start_leader.sh` 使用 `enable_downlink=false`。串口看门狗和节点析构均不会发送 11 字节控制帧，因此不会把主车从自主模式切到 `_APP_Control`。

## 4. 构建和启动

三台 K7 使用同一分支并构建：

```bash
cd ~/K7_ros2
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=5
```

启动顺序：

```bash
# leader
./run/start_leader.sh

# follower1
./run/start_follower.sh follower1 1.2 /leader/odom

# follower2
./run/start_follower.sh follower2 1.2 /follower1/odom
```

三台 `ROS_DOMAIN_ID` 必须相同，namespace 必须不同。学校或公司 WiFi 若屏蔽组播，改用同一路由器/AP 热点或有线交换机。

## 5. 控制和安全逻辑

跟随控制每 20 Hz 运行：

```text
v = predecessor_v + kp_gap * (path_gap - target_gap)
w = v * path_curvature + k_heading * heading_error + k_lateral * lateral_angle
w += local_apf_turn
(v, w) -> differential-drive joint limit -> cmd_vel
```

联合限幅保证：

```text
v_left  = v - w * track_width / 2 >= 0
v_right = v + w * track_width / 2 >= 0
max(v_left, v_right) <= v_max
```

当前默认值：`v_max=0.4 m/s`、`w_max=1.2 rad/s`、`track_width=0.329 m`。测距小于 `0.7 m` 开始减速和转向，小于 `0.18 m` 停车。前车、本车 odom 或测距数据超过 0.5 秒未更新都会停车。

注意：三路前向传感器看不到车后方和侧后方；APF 只适合低速局部避障，不能替代全向安全检测。

## 6. 第一轮测试

1. 架空从车驱动轮，确认 `/followerN/cmd_vel` 有输出，遮挡左右传感器时角速度方向正确。
2. 只运行 leader + follower1，先设 `v_max:=0.20`，在无障碍直线上测试起停和间距。
3. 给 follower1 单独放置软障碍，确认车辆减速后边走边转，两轮不反转。
4. 加入 follower2，再测试 leader 绕障后的轨迹传递。
5. 最后逐步提高速度，并用 `ros2 topic hz` 检查三个 odom 与测距话题频率。

键盘/手柄在 twist_mux 中优先级高于 follower，可用于接管。急停仍应保留独立硬件手段。

## 7. 参数位置

- 跟随与 APF：`src/k7_follower/config/follower.yaml`
- 从车启动：`src/k7_follower/launch/k7_follower.launch.py`
- 串口轻量模式：`src/k7_bringup/launch/k7_core.launch.py`
- 仲裁优先级：`src/k7_bringup/config/twist_mux.yaml`
- 脚本：`run/start_leader.sh`、`run/start_follower.sh`
