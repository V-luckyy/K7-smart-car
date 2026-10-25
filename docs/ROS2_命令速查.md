# K7 常用命令

以下在 K7 的 Bash 执行，假定工作区为 `~/Code/K7_ros2`，车辆 namespace 为 `car1`。命令与当前源码接口对应，尚未在目标板执行验收。完整定义见 [架构手册](K7_开发架构方案.md)。

## 1. 环境与构建

```bash
cd ~/Code/K7_ros2
source /opt/ros/jazzy/setup.bash
colcon list
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

新终端也要 source 系统与本工作区。`colcon list` 应只有架构手册列出的 7 个自有包；旧算法包和节点已从源码删除。出现旧包/旧节点时，按[升级清理](K7_上手操作指南.md#2-k7-软件部署)替换旧 src、清理 build/install/log 并使用新终端，不要混用历史 overlay。任务统一用 launch，已删除旧 run/ 脚本。

断电后先保留日志、检查磁盘空间和源码，再尝试 `colcon build --symlink-install --cmake-clean-cache --parallel-workers 2`。仍有旧目标时，可在确认当前目录后将 `build`、`install`、`log` 重命名备份，在只 source 系统 ROS 的新终端重建；不要删除源码。日志中的 undefined reference 不能仅凭“曾断电”判为硬件故障。

## 2. 唯一启动入口

所有车辆都使用 `k7_bringup/launch/robot.launch.py` 启动。参数以 `参数名:=值` 传入；每辆车各启动一个 launch 实例，并用不同的 `namespace` 隔离节点和话题。launch 启动后任务状态为 IDLE，不会自动行驶。

| 参数 | 默认值 | 用途 |
|---|---|---|
| `profile` | `trajectory` | 车辆任务类型：`manual` 手动遥控、`trajectory` 单车路径、`formation` 跟随前车。 |
| `algorithm` | `apf_stanley` | 控制算法。当前只实现 `apf_stanley`；无感知测试时控制器自动走 Stanley-only 分支。 |
| `namespace` | `car1` | 本车 ROS 命名空间。多车必须各自使用唯一名称，例如 `car1`、`car2`。 |
| `config` | 安装目录中的 `robot.yaml` | 本车配置文件路径。多车可分别指定 YAML；使用自定义文件时传绝对路径。 |
| `test_mode` | `none` | `none` 为正常感知模式；`no_perception` 仅适用于 `trajectory` 和 `formation`，用于无相机低速功能验证。 |
| `publish_fleet` | `false` | 是否发布本车 `fleet/state`。需要其他车辆跟随的主车必须设为 `true`。 |
| `record` | `false` | 是否在 launch 后立即开始写实验数据。默认不写盘；运行中仍可通过记录服务开始或停止。 |

```bash
ros2 launch k7_bringup robot.launch.py profile:=trajectory namespace:=car1
# 手动模式，默认不记录
ros2 launch k7_bringup robot.launch.py profile:=manual namespace:=car1
# 自定义配置，启动即记录；仍不会自动行驶
ros2 launch k7_bringup robot.launch.py profile:=trajectory namespace:=car1 config:=$HOME/k7/car1.yaml record:=true
# 无感知低速单车路径测试；仅限封闭空旷场地
ros2 launch k7_bringup robot.launch.py profile:=trajectory namespace:=car1 test_mode:=no_perception
```

以上是不同启动示例，不是要同时执行的命令。单车给定路径用 `profile:=trajectory`；无感知验证可附加 `test_mode:=no_perception`。多车编队中主车通常用 `profile:=trajectory publish_fleet:=true`，跟随车用 `profile:=formation`；如需无感知验证，每辆车的启动命令都要分别附加 `test_mode:=no_perception`。编队坐标、启动顺序及完整命令见[编队说明](多机编队_LeaderFollower_说明.md)。

默认 `algorithm:=apf_stanley`，只有此算法已接入；未知算法和不支持的参数组合会拒绝启动。换 profile、算法或参数前，必须先停车确认，再退出并重启。

## 3. 启动前检查

```bash
ros2 node list
ros2 action list -t
ros2 service list -t
ros2 topic echo /car1/task/status --once
ros2 topic echo /car1/control/status --once
ros2 topic echo /car1/perception/scan --once
ros2 topic echo /car1/odom --once
ros2 topic info /car1/cmd_vel --verbose
```

`cmd_vel` 只能有 `command_guard` 一个发布者。正常模式下标定未确认、扇区未知/过近、反馈过期或底盘禁能时不授权。不能靠把 `calibration_confirmed` 改成 true 绕过实际标定。

`test_mode:=no_perception` 可用于 `trajectory` 单车路径或 `formation` 编队。它不启动相机/双目节点，控制器仅运行 Stanley，guard 不检查 scan，并对最终线速度限幅到 0.05 m/s。此模式没有避障和近障停车能力，只能在封闭空旷场地测试并安排现场急停；正常运行不要启用。每辆车都要单独带上此参数。

## 4. 单车路径 Action

仅在低速验收区域、起点和朝向与下例一致时执行。启动 `trajectory` 后，下面路径是 `car1/odom` 中从 (0,0) 沿 +x 到 (2,0)，目标速度 0.10 m/s：

```bash
ros2 action send_goal /car1/task/follow_path k7_msgs/action/FollowPath "{path: {header: {frame_id: car1/odom}, poses: [{pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}]}, target_speed: 0.10, position_tolerance: 0.10, yaw_tolerance: 0.15}" --feedback
```

坐标按当前 odom 重设后的原点解释，不能直接复用上次运行的坐标。路径末点四元数表示期望终点航向；非直线路径应由任务客户端生成，不手工堆长命令。Action 客户端可调用标准取消；终端退出本身不作为可靠停车命令。

## 5. 手动速度

启动 `manual`。先在另一终端持续发送带时间戳的零速，再授权，避免授权后 0.25 s 没有遥控输入而退出：

```bash
ros2 topic pub /car1/teleop/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: car1/base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" -r 20
# 另一终端
ros2 service call /car1/task/set_manual std_srvs/srv/SetBool "{data: true}"
```

实际遥控客户端应持续 20 Hz 更新消息，松开使能时发零，不能只在按键变化时发。允许前进并同时转向；不支持倒车、原地旋转。手动模式不自动 APF 绕障，仍保留近障停车。不要向 `cmd_vel`、`control/candidate` 或内部授权服务直接注入命令。

## 6. 停车与恢复

```bash
ros2 service call /car1/task/stop std_srvs/srv/Trigger "{}"
ros2 topic echo /car1/task/status
# 软件锁定停车
ros2 service call /car1/control/hold std_srvs/srv/SetBool "{data: true}"
# 排除故障后解除锁定；不会恢复旧任务
ros2 service call /car1/control/hold std_srvs/srv/SetBool "{data: false}"
ros2 service call /car1/task/stop std_srvs/srv/Trigger "{}"
```

`task/stop` 返回 success 仅表示受理。必须看到新鲜 odom 连续静止、`control/status` 的 stopped=true，以及 `task/status` 回到 IDLE，才能重新发任务或换配置。FAULT 不能当作已停车。服务不可用时用现场硬件停机手段；软件 hold 不是硬件急停。

## 7. 实验记录

```bash
ros2 service call /car1/experiment/record std_srvs/srv/SetBool "{data: true}"
ros2 service call /car1/experiment/record std_srvs/srv/SetBool "{data: false}"
```

记录器节点始终启动，默认不写盘。用 `record:=true` 可在 launch 后立即开始记录；否则可通过上面的服务在运行中开始或停止。配置 `experiment_recorder.variables / topics / rate`，调参后先停车重启 launch 再记录。字段与目录见 [日志说明](../logs/README.md)。不要在车辆活动时回放控制话题，也不要默认录全量图像。

## 8. 频率、配置和排错

```bash
ros2 topic hz /car1/odom
ros2 topic hz /car1/perception/scan
ros2 topic hz /car1/cmd_vel
ros2 topic bw /car1/fleet/state
ros2 param get /car1/controller_node stanley_gain
ros2 param dump /car1/controller_node
ros2 interface show k7_msgs/action/FollowPath
ros2 interface show k7_msgs/action/FollowLeader
ros2 interface show k7_msgs/msg/FleetState
ros2 node info /car1/command_guard
```

自有节点参数只读，更改统一 YAML 后重启，不使用 `ros2 param set` 进行半生效调参。图像带宽检查和远程图像订阅本身会增加负载，测完退出。编队命令见 [编队说明](多机编队_LeaderFollower_说明.md)。

## 9. 停机采集与离线双目检查

以下命令从 `~/Code/K7_ros2` 执行，先停止车辆 launch，避免占用相机。`--dev` 是核对后的 video 数字编号；左右图片与本车标定路径替换成实际位置。

```bash
python3 src/k7_camera/scripts/capture_calibration.py --help
python3 src/k7_camera/scripts/capture_calibration.py --dev 73 --out "$HOME/calibration_images"
mkdir -p "$HOME/stereo_check"
python3 src/k7_camera/scripts/stereo_match.py --left "$HOME/calibration_images/left/left_000.png" --right "$HOME/calibration_images/right/right_000.png" --config-dir src/k7_camera/config --out-dir "$HOME/stereo_check"
```

采集工具自带 8888 端口预览，启动后按终端提示采集；不完成标定求解。离线匹配使用 OpenCV contrib 的 WLS，不是运行节点；运行时通过统一配置 `stereo_obstacles.publish_depth` 开关深度输出，不另起旧 depth/splitter 节点。
