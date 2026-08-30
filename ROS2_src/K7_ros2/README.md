# K7_ros2 — K7 智能小车 ROS2 (Jazzy) 工作区

> 架构方案见仓库根目录 `docs/K7_开发架构方案.md`，硬件事实见 `../../CLAUDE.md` 第 13 节。

## 包结构

| 包 | 类型 | 说明 |
|----|------|------|
| `k7_bringup` | ament_cmake (C++) | STM32 串口底盘节点（移植自 wheeltec，含 cmd_vel 看门狗）+ EKF/Madgwick 配置 + udev 规则 + 核心 launch |
| `k7_description` | ament_cmake | 小车 URDF（两轮差速 + 万向轮 + 双目相机 frame） |
| `k7_camera` | ament_python | LRCP 2MV 双目 splitter + 标定文件 + 相机 launch（Phase 3） |
| `k7_mpc` | ament_python | 番外线：MPC 避障实车验证（仿真 five_version_progressive_r04_with_pid V1 控制器移植，odom + 3 路红外(/ir_distances) → `/cmd_vel_mpc`） |
| `k7_msgs` | ament_cmake | 自定义消息（当前仅 `IrDistances`：三路红外测距 front/left45/right45，单话题 `/ir_distances`） |

`k7_nav`（Phase 4）、`k7_npu`（Phase 5）届时再建。

## 话题一览

> 运行 `ros2 launch k7_bringup k7_core.launch.py`（串口 + EKF + Madgwick + robot_state_publisher）后的话题。
> `k7_mpc` 节点单独运行时，额外多一个 `/cmd_vel_mpc`（见文末）。

### 一、底盘上行数据（`k7_serial_node` 发布，STM32 上行 20Hz）

| 话题 | 类型 | 内容 | 频率 |
|------|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 编码器积分里程计：位置(x,y,θ) + 线速度 + 角速度 | 20Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | MPU6050 原始：四元数姿态 + 角速度 + 线加速度 | 20Hz |
| `/ir_distances` | `k7_msgs/IrDistances` | 三路红外测距 `front`/`left45`/`right45`（米） | 20Hz |
| `/PowerVoltage` | `std_msgs/Float32` | 底盘电池电压（V） | ~2Hz |
| `/robot_charging_flag` | `std_msgs/Bool` | 是否充电 | 20Hz* |
| `/robot_charging_current` | `std_msgs/Float32` | 充电电流（A） | 20Hz* |
| `/robot_red_flag` | `std_msgs/UInt8` | 红外对接信号（充电桩） | 20Hz* |

\* 充电话题在收到回充帧时发布；红外传感器在线时固件也会顺带发回充帧（值全 0）。

### 二、底盘指令（`k7_serial_node` 订阅）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令：`linear.x`=线速度(m/s)，`angular.z`=角速度(rad/s) |
| `/red_vel` | `geometry_msgs/Twist` | 红外对接速度 |
| `/robot_recharge_flag` | `std_msgs/Int8` | 回充标志位 |

### 三、处理后数据（EKF / Madgwick）

| 话题 | 类型 | 内容 |
|------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | Madgwick 滤波后姿态 |
| `/odom_combined` | `nav_msgs/Odometry` | EKF 融合 odom+imu（**MPC 的状态源**） |

### 四、机器人模型 / TF

| 话题 | 类型 | 内容 |
|------|------|------|
| `/robot_description` | `std_msgs/String` | URDF 描述（启动时发一次） |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（robot_state_publisher） |
| `/tf` | `tf2_msgs/TFMessage` | 动态坐标变换（odom_combined→base_footprint） |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态变换（base_footprint→base_link / gyro_link） |

### 五、系统 / 诊断

| 话题 | 类型 | 内容 |
|------|------|------|
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | 参数变更事件（系统自动） |
| `/rosout` | `rcl_interfaces/Log` | 日志聚合（系统自动） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息（robot_localization 等） |

### 服务（非话题）

| 服务 | 类型 | 用途 |
|------|------|------|
| `/set_pose` | `robot_localization/srv/SetPose` | 手动设置 EKF 位姿 |

> **MPC 运行时**（`ros2 launch k7_mpc k7_mpc.launch.py`）：额外订阅 `/odom_combined`、`/ir_distances`，发布 `/cmd_vel_mpc`（`geometry_msgs/Twist`）。

## 依赖安装（K7 上执行）

一键脚本（推荐）——检测并安装 ROS2 Jazzy（已装则跳过）→ apt 安装全部项目依赖 → 配置 `~/.bashrc` 自动 source：

```bash
cd ~/Code/K7_ros2
./setup_env.sh
```

> 网络慢可先把 apt 源换成国内镜像（如清华源）再运行。
> 也可以用 rosdep 按 package.xml 自动装：`rosdep install --from-paths src -y --ignore-src`

## 摄像头测试（在 K7 上执行）

`k7_camera/scripts/camera_server.py` 是从板子 `~/Videos/camera_server.py` 同步进来的**独立调试脚本**，用于在 PC 浏览器里查看 LRCP 2MV 画面，**不是项目正式功能节点**。

```bash
cd ~/Code/K7_ros2/src/k7_camera
python3 scripts/camera_server.py
```

然后在 PC 浏览器打开 `http://192.168.10.2:8888/`（替换成实际 K7 IP），即可看到 3840×1080 的左右并排双目画面。

> 注意：它直接打开 `/dev/video73`；后续 Phase 3 会用 `usb_cam` + splitter 节点走标准 ROS 图像话题。

```bash
cd ~/Code/K7_ros2
colcon build --symlink-install
source install/setup.bash
```

## udev 规则（K7 上执行一次）

```bash
cd ~/Code/K7_ros2/src/k7_bringup/udev
./k7_udev.sh        # 安装规则并重新加载，之后重新插拔 USB
ls -l /dev/k7_controller   # 应指向 ttyUSB*
```

## 开发工作流（Windows ↔ K7）

代码主副本在本仓库（Windows），同步到 K7 构建运行：

```bash
# Windows Git Bash 下（在本仓库根目录执行）：
# 源路径末尾【不带斜杠】，才会在板子生成 ~/Code/K7_ros2/（工作区根目录）
scp -r ROS2_src/K7_ros2 kickpi@<K7_IP>:~/Code/

# K7 上重新构建（在含 src/ 的工作区根目录下执行；必要时先删 build/ install/ log/）
cd ~/Code/K7_ros2
colcon build --symlink-install
```

VSCode Remote-SSH 用于板上编译调试；大改动回本仓库提交，避免两边漂移。
