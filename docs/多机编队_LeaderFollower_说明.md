# K7 多机编队（Leader–Follower）说明

> 建立时间：2026-09-06
> 适用范围：3 台 K7（RK3576 + C50X 底盘），WiFi/以太网同一局域网
> 初版方案：**速度复刻 + 纵向间距保持**（v1，最简）。`k7_follower` 包在 `ROS2_src/K7_ros2/src/`。

---

## 一、总体思路

三台车**跑同一份 `K7_ros2` 代码**，靠 **ROS_DOMAIN_ID + namespace** 区分彼此，谁都不用改源码。

| 角色 | namespace | 跑的 launch | 作用 |
|------|-----------|-------------|------|
| **leader**（本板） | `leader` | `k7_core` + `k7_twist_mux` | 正常底盘链路，发布自己融合位姿 `/leader/odom_combined`；速度源随意（键盘/手柄/固件 APF） |
| **follower1** | `follower1` | `k7_follower.launch.py`（内含 core + twist_mux + 跟随节点） | 订阅 leader 位姿，算速度去跟踪 |
| **follower2** | `follower2` | 同上 | 同上 |

> 三块板 ROS2 各自跑自己的 `k7_serial_node`（连自己的 STM32）。多机不需要额外中枢，DDS 在同一局域网组播即可互相发现。

### 数据流（以 follower1 为例）

```
leader 板:  STM32→串口→EKF → /leader/odom_combined ──────────────┐ (WiFi, DDS)
follower1:                                                        │
   /leader/odom_combined (跨机订阅) ─┐                             │
   follower1/odom_combined(自身里程) ─┼→ follower_node → cmd_vel_follower
   follower1/ir_distances(自身红外) ─┘        │ (twist_mux 仲裁, 优先级70)
                                             ↓
                                 follower1/cmd_vel → 串口 → follower1 的 STM32 → 电机
```

---

## 二、为什么是 namespace（而不是"leader 一个工程、follower 另一个工程"）

三台车跑同一套底层（串口/EKF/TF/twist_mux），话题名如果相同（`/odom`、`/cmd_vel`…）会互相覆盖：
follower 会收到自己 + leader + 另一台 follower 混在一起的数据。给每台车 namespace 后：

```
单机（默认，无 namespace）： /odom_combined   /cmd_vel
leader（namespace:=leader）：/leader/odom_combined   /leader/cmd_vel
follower1：                /follower1/odom_combined /follower1/cmd_vel
```

话题表（编队实际用到的）：

| 话题 | 位置 | 类型 | 说明 |
|------|------|------|------|
| `/leader/odom_combined` | leader | `nav_msgs/Odometry` | leader 融合位姿（位置+速度）——跟随者的参考输入 |
| `/followerN/odom_combined` | follower | `nav_msgs/Odometry` | 从机自己的里程，控制器取自身 `v` 做间距积分 |
| `/followerN/ir_distances` | follower | `k7_msgs/IrDistances` | 从机三路红外，`front` 用于防追尾安全刹停 |
| `/followerN/cmd_vel_follower` | follower | `geometry_msgs/Twist` | 跟随节点输出，进 twist_mux 仲裁 |
| `/followerN/cmd_vel` | follower | `geometry_msgs/Twist` | twist_mux 仲裁输出 → 底盘串口 |

twist_mux 各速度源优先级（`config/twist_mux.yaml`）：joy(100) > 键盘(80) > **follower(70)** > mpc(50)。
从机运行时只有 follower 这一路在发；键盘/手柄优先级更高，随时可以"抢"回控制权，停 0.5s 后跟随自动恢复。

---

## 三、代码改动清单（本次已改好）

| 文件 | 改动 |
|------|------|
| `k7_bringup/launch/k7_core.launch.py` | 加 `namespace` 参数（默认空），全部节点包进命名空间 |
| `k7_bringup/launch/k7_twist_mux.launch.py` | 加 `namespace` 参数 |
| `k7_bringup/config/ekf.yaml` | `imu0` 改为相对话题名 `imu/data_raw`（原来带 `/` 是绝对名，namespace 下会找不到） |
| `k7_bringup/config/twist_mux.yaml` | 加 `follower` 输入（`cmd_vel_follower`，优先级 70） |
| `k7_follower/`（**新包**） | 跟随节点 + launch + 参数 |
| `run/start_leader.sh` / `run/start_follower.sh` | 一键启动脚本 |

> 单机用法完全不受影响：不传 `namespace:=` 时行为和以前一模一样（namespace 默认空串）。

---

## 四、部署到从机板（你手动做一次）

从机是两块独立的 K7，需要把整份 `K7_ros2` 拷过去并编译。任选一种：

**方式 A：git 同步**（三块板都从仓库拉同一分支）
```bash
# 每块板上：
cd ~/ros2_ws   # 换成你实际的 K7_ros2 上级目录
git fetch origin && git checkout feat/k7-mpc && git pull
cd K7_ros2
colcon build --symlink-install
```

**方式 B：rsync/scp 拷贝**（不想每块板都走 git）
```bash
# 在 leader 开发机/板上，把改好的工作区推到两块从机（IP 举例 192.168.1.11/.12）：
rsync -av --exclude 'build' --exclude 'install' --exclude 'log' \
      K7_ros2/ kickpi@192.168.1.11:~/K7_ros2/
# 然后从机板：colcon build --symlink-install
```

**编译前确认从机已装依赖**（一般建工作区时已装过，缺什么装什么）：
```bash
sudo apt install -y ros-jazzy-twist-mux ros-jazzy-robot-localization \
                    ros-jazzy-imu-filter-madgwick
```

**每台车烧录模式提醒（重要，见"七、固件前置"）。**

---

## 五、快速开始（三台车都在同一个 WiFi/交换机下）

把三台车按 leader 在前、间距≈目标间距（默认 1.2m）摆成一列，然后：

```bash
# ── leader 板 ──
cd K7_ros2
./run/start_leader.sh
# 再开一个终端给 leader 一个速度源，例如键盘：
ros2 run k7_bringup ...    # 见你的键盘/手柄启动方式（发 /leader/cmd_vel_key）

# ── follower1 板 ──
cd K7_ros2
./run/start_follower.sh follower1        # 间距默认 1.2

# ── follower2 板 ──
./run/start_follower.sh follower2 1.5    # 想让 2 号跟远一点就传第二个参数
```

验证是否连通（任意一块板上，能看到 leader 数据就说明 DDS 通了）：
```bash
ros2 topic echo /leader/odom_combined --once
```

### ROS_DOMAIN_ID
三块板必须一致（默认都 0）。如果不一致：
```bash
export ROS_DOMAIN_ID=5
```
（三台都设同一个数；网络里有其它 ROS2 设备时建议用非 0 号避开。）

---

## 六、跟随节点算法与参数

核心文件 `k7_follower/k7_follower/convoy_follower.py`：

1. **角速度复刻**：`w = w_leader`（限幅）。leader 转弯跟着转，保持队形朝向一致。
2. **线速度保持间距**：间距 `gap` 由 `d(gap)/dt = v_leader − v_follower` 积分得到；
   线速度 `v = v_leader + kp·(gap − target_gap)`，远了加速、近了减速。
3. **前红外安全**：`front < ir_safe(0.35m)` 立即停（防追尾 / 防撞）；`< ir_slow(0.6m)` 限速。
4. **断链保护**：超过 `leader_timeout(1s)` 没收到 leader 数据 → 发零速（这点对应 K7 固件"断链不停车"，必须在 ROS 层兜底）。

参数都在 `k7_follower/config/follower.yaml`，运行时也能改（不用重启）：
```bash
ros2 param set /follower1/follower_node target_gap 1.5   # 动态改间距
ros2 param set /follower1/follower_node kp_gap 1.0       # 动态改回位强度
```

| 参数 | 默认 | 含义 | 建议 |
|------|------|------|------|
| `target_gap` | 1.2 | 目标纵向间距 (m) | 小车刹车距离有限，别设太近（≥0.8） |
| `kp_gap` | 0.8 | 间距回位增益 | 越大回位越快，过大会前后振荡 |
| `v_max` | 0.5 | 从机最大线速度 | 应 ≤ leader 常用速度，否则追尾风险 |
| `w_max` | 1.2 | 从机最大角速度 | |
| `leader_timeout` | 1.0 | leader 断链急停时间 (s) | 别设太大，安全第一 |
| `ir_safe` / `ir_slow` | 0.35 / 0.6 | 前红外安全距离 (m) | |

**已知局限（v1）**：间距靠"两车速度差积分"的一维近似，leader 转弯时从机走内道/外道会带来少量间距误差；且各车 odom 原点不统一，做不了严格的"沿 leader 轨迹几何位置跟踪"。要更准有两种升级路线：
- **统一坐标系**：三车开机摆成已知相对位姿，用 `robot_localization` 的 `set_pose` 注入初始位姿，把三车 odom 对齐到同一世界系 → follower 就能对 leader 的 (x,y,θ) 做位置闭环（直线编队最稳）。
- **相对测量**：从机用双目/ToF 测 leader 相对自身的方位距离做闭环（工作量最大，但最接近真实编队）。

---

## 七、固件前置（必须注意，否则从机"不听话"）

`balance_task` 的控制模式语义：
- **ROS 模式**：串口收到 11 字节速度帧 → 固件切 `_APP_Control`，用帧里的 Vx/Vz 驱动电机（此时 `robot_control` 里 APF_task 写的那套**被忽略**）。
- **APF 独立模式**：`apf_task.c` 每周期强写 `robot_control.Vx/Vz` 且 `ControlMode=0`，`balance_task` 走直驱分支 → **无视串口 cmd_vel**。

所以：
- **leader**：想让它自己按 APF+Stanley 圆跑，就烧带 APF_task 的固件（当前主线固件即可），它照样把实际速度报给 `/leader/odom_combined`，从机跟的就是"它真的在跑的速度"——很自然的演示。
- **follower1/2**：**必须烧"ROS 模式"固件**（收串口 cmd_vel），否则它自己的 APF 会抢着开车、无视编队节点下发的速度。做法：`USER/main.c` 里把 `CreateTaskChecked(APF_task, ...)` 那一行注释/条件编译掉，Keil 重编烧录（或专门编一个 ROS 模式 target）。
- 从机测试编队前，确认没在跑会抢 `/followerN/cmd_vel` 的 MPC/键盘速度源。

---

## 八、常见问题排查

| 现象 | 检查 |
|------|------|
| 从机 `ros2 topic echo /leader/odom_combined` 收不到 | ①三台 `ROS_DOMAIN_ID` 是否一致；②是否同一网段；③学校/公司 WiFi 若禁组播，DDS 无法跨机发现，改用 AP 热点或交换机 |
| 收到 leader 但车不动 | `ros2 topic echo /follower1/cmd_vel_follower` 看跟随节点有没有输出；`ros2 topic echo /follower1/cmd_vel` 看 twist_mux 有没有仲裁；确认从机 STM32 是 ROS 模式固件 |
| 从机只转不走 / 抖动 | `kp_gap` 过大或 `target_gap` 过小；把 leader 摆正后 `ros2 param set /followerN/follower_node gap_reset`…（重启节点即重置间距积分到 target）|
| 一启动就往一个方向转 | 左右轮编码器方向/接线，或用前红外挡住从机正前方看它是否刹停（验证 IR 安全层） |
| 转弯时跟丢 | v1 已知局限（见六），转弯半径小时放大间距 |
| 各车 topic 又混了 | 检查三台 launch 是否都传了 `namespace:=`，且各不相同 |

---

## 九、文件与代码位置速查

| 想看什么 | 去哪个文件 |
|---------|-----------|
| 跟随算法（速度复刻/间距积分/红外安全） | `k7_follower/k7_follower/convoy_follower.py` |
| 跟随参数调优 | `k7_follower/config/follower.yaml` |
| 从机整车启动 | `k7_follower/launch/k7_follower.launch.py` |
| namespace 支持（底盘） | `k7_bringup/launch/k7_core.launch.py`、`k7_twist_mux.launch.py` |
| twist_mux 仲裁 | `k7_bringup/config/twist_mux.yaml` |
| leader / follower 一键启动 | `run/start_leader.sh`、`run/start_follower.sh` |
