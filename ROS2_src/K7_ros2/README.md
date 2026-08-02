# K7_ros2 — K7 智能小车 ROS2 (Jazzy) 工作区

> 架构方案见仓库根目录 `docs/K7_开发架构方案.md`，硬件事实见 `../../CLAUDE.md` 第 13 节。

## 包结构

| 包 | 类型 | 说明 |
|----|------|------|
| `k7_bringup` | ament_cmake (C++) | STM32 串口底盘节点（移植自 wheeltec，含 cmd_vel 看门狗）+ EKF/Madgwick 配置 + udev 规则 + 核心 launch |
| `k7_description` | ament_cmake | 小车 URDF（两轮差速 + 万向轮 + 双目相机 frame） |
| `k7_camera` | ament_python | LRCP 2MV 双目 splitter + 标定文件 + 相机 launch（Phase 3） |
| `k7_mpc` | ament_python | 番外线：MPC 避障实车验证（仿真 five_version_progressive_r04_with_pid V1 控制器移植，odom + 3 路红外 Range → `/cmd_vel_mpc`） |

`k7_nav`（Phase 4）、`k7_npu`（Phase 5）届时再建。

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
scp -r ROS2_src/K7_ros2/ kickpi@<K7_IP>:~/Code/
# K7 上重新构建（必要时先删 build/ install/ log/）
```

VSCode Remote-SSH 用于板上编译调试；大改动回本仓库提交，避免两边漂移。
