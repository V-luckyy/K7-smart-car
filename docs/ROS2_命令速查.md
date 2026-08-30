# ROS2 命令速查（K7 板端）

> ROS2 Jazzy + Ubuntu 24.04，工作区在板子 `~/Code/K7_ros2/`。

## 0. 每个新终端先 source

```bash
source /opt/ros/jazzy/setup.bash             # 系统 ROS2（装好后 .bashrc 已自动加）
source ~/Code/K7_ros2/install/setup.bash     # 本项目工作区（colcon build 后）
```

## 1. 构建

```bash
cd ~/Code/K7_ros2
colcon build --symlink-install                              # 全量构建
colcon build --symlink-install --packages-select k7_mpc     # 只建单个包（改哪包建哪包）
source install/setup.bash                                   # 建完重新 source
```

> `--symlink-install`：改 Python（k7_mpc/k7_camera）不用重新 build；改 C++（k7_bringup/k7_msgs/k7_description）才需要。

## 2. 运行（本项目有哪些包 / launch / 节点 / 参数）

```bash
ros2 launch <包> <launch文件>                    # 启动 launch
ros2 run <包> <节点>                             # 单独起一个节点
ros2 run <包> <节点> --ros-args -p 参数:=值       # 带参数起
```

> `ros2 launch` 第二个参数只写**文件名**、不写路径：`ros2 launch k7_bringup k7_core.launch.py`（自动到包安装后的 `share/k7_bringup/launch/` 里找）。要直接跑源码路径也行：`ros2 launch ~/Code/K7_ros2/src/k7_bringup/launch/k7_core.launch.py`。

### 包清单（5 个，源码都在 `~/Code/K7_ros2/src/`）

| 包 | 语言 | 作用 |
|----|------|------|
| `k7_bringup` | C++ | 底盘串口节点 + EKF/Madgwick/twist_mux 配置 + 核心 launch |
| `k7_camera` | Python | 双目拆分节点 + 双目标定 yaml |
| `k7_description` | C++ | URDF 模型（`k7_robot.urdf`） |
| `k7_mpc` | Python | MPC 避障控制器（番外线） |
| `k7_msgs` | C++ | 自定义消息（`IrDistances`） |

### launch 文件（3 个）

| launch | 作用 |
|--------|------|
| `k7_bringup/launch/k7_core.launch.py` | 串口 + TF + EKF + Madgwick（跑车必起） |
| `k7_bringup/launch/k7_twist_mux.launch.py` | twist_mux 速度仲裁（MPC 需要） |
| `k7_mpc/launch/k7_mpc.launch.py` | MPC 控制器 |

### 节点 / 可执行（`ros2 run <包> <节点>` 直接能起）

| 节点 | 包 | 作用 |
|------|----|------|
| `k7_serial_node` | k7_bringup | 底盘串口节点 |
| `mpc_node` | k7_mpc | MPC 控制器 |
| `fake_ir_publisher` | k7_mpc | 假红外发布（联调控制链用） |
| `stereo_splitter` | k7_camera | 双目图拆分 |

### 参数在哪看

- **运行时 ROS 参数**（`ros2 param list` 能看到）：串口端口/波特率在 `k7_core.launch.py`；MPC 的 `version` 在 `k7_mpc.launch.py`（也可 `ros2 param set` 临时改）。
- **算法/滤波常量**（改源码后需重跑，Python 改 `.py` 即时生效，改 C++ 或 yaml 要重 build）：
  - EKF / Madgwick / twist_mux：`k7_bringup/config/ekf.yaml`、`imu.yaml`、`twist_mux.yaml`
  - MPC 算法常量：`k7_mpc/k7_mpc/mpc_lib/common_config.py`；话题/路径：`mpc_config.py`
  - 双目标定：`k7_camera/config/left.yaml`、`right.yaml`

## 3. 看话题

```bash
ros2 topic list                              # 所有话题
ros2 topic list -t                           # 话题 + 消息类型
ros2 topic info /cmd_vel                     # 某话题的发布者/订阅者/QoS
ros2 topic echo /ir_distances                # 打印数据（Ctrl+C 停）
ros2 topic echo /cmd_vel --once              # 只打一帧
ros2 topic hz /odom_combined                 # 测发布频率
ros2 topic bw /camera/image_raw              # 测带宽
```

发布指令（调试底盘用）：
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

## 4. 节点 / 参数 / 服务

```bash
ros2 node list                               # 运行中的节点
ros2 node info /k7_mpc_node                  # 节点订阅/发布/服务

ros2 pkg list                                # 装了哪些包
ros2 pkg executables k7_mpc                  # 某包有哪些可运行节点
ros2 pkg prefix k7_bringup                   # 某包安装路径（launch/config 在这）

ros2 param list                              # 所有参数
ros2 param get /k7_mpc_node version          # 读参数
ros2 param set /k7_mpc_node version V1       # 临时改参数（重启失效）

ros2 service list                            # 所有服务
```

## 5. 消息定义

```bash
ros2 interface show k7_msgs/msg/IrDistances  # 看自定义消息字段
ros2 interface list | grep k7_msgs           # 本项目有哪些自定义消息
```

## 6. 数据记录与分析

```bash
# 录包（离线回放分析）
ros2 bag record /odom_combined /ir_distances /cmd_vel_mpc   # 录指定话题
ros2 bag record -a -o my_bag                  # 录全部，存 my_bag/
ros2 bag info my_bag                          # 看包信息
ros2 bag play my_bag                          # 回放

# 话题实时存文本
ros2 topic echo /ir_distances > ir.txt        # 重定向到文件
```

> MPC 节点自带 CSV 日志（`~/mpc_log/mpc_*.csv`），跑车时优先看它，一般不用手动录包。
> 轨迹绘图脚本：`src/k7_mpc/scripts/plot_trajectory.py`（实时刷新/离线回放/导出 PNG，纯 Python+matplotlib）。

## 7. 多终端（tmux）

SSH 里开多个"窗口"用 tmux，比连多条 SSH 省事：

```bash
tmux                                    # 新建会话
tmux new -s ros                         # 命名会话
tmux attach -t ros                      # 回到会话（掉线后）
tmux ls                                 # 列出会话
```

会话内快捷键（先按 `Ctrl+b` 再按下面键）：

| 键 | 作用 |
|----|------|
| `c` | 新开窗口 |
| `%` | 左右分屏 |
| `"` | 上下分屏 |
| `o` | 切到下一分屏 |
| `d` | 脱离会话（后台继续跑） |
| `[` | 翻页查看（退出按 `q`） |

## 8. 排错

```bash
ros2 doctor                                   # 环境体检
ros2 topic info /ir_distances --verbose       # 查 QoS 是否匹配（收发对不上时看这个）
ros2 run rqt_graph rqt_graph                  # 节点/话题关系图（需图形界面）
ros2 run rqt_console rqt_console              # 按级别看日志
ros2 run tf2_tools view_frames                # 生成 TF 树 PDF
```

## 9. 本项目关键话题速查

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | `Odometry` | 编码器里程计（20Hz） |
| `/odom_combined` | `Odometry` | EKF 融合（**MPC 状态源**） |
| `/imu/data_raw` | `Imu` | MPU6050 原始 |
| `/ir_distances` | `IrDistances` | 三路红外 front/left45/right45（米） |
| `/cmd_vel` | `Twist` | 底盘速度指令（serial_node 订阅） |
| `/cmd_vel_mpc` | `Twist` | MPC 输出 → twist_mux → `/cmd_vel` |

## 10. 本项目跑车标准流程（3 个终端）

```bash
# 终端1：底盘串口 + TF + EKF + Madgwick
ros2 launch k7_bringup k7_core.launch.py

# 终端2：twist_mux 速度仲裁（/cmd_vel_mpc → /cmd_vel）
ros2 launch k7_bringup k7_twist_mux.launch.py

# 终端3：MPC 控制器（默认 V5）
ros2 launch k7_mpc k7_mpc.launch.py
```

> 换版本：`ros2 launch k7_mpc k7_mpc.launch.py version:=V1`（V5 为默认，V1 回退对比）。

---

> 常用组合：起 3 终端 → `ros2 topic echo /ir_distances` 看红外数据 → `ros2 topic hz /cmd_vel_mpc` 看下发频率 → 回看 `~/mpc_log/` 里 CSV 做离线分析。
