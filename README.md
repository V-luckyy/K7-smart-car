# K7 智能小车 — RK3576 项目

> KICKPI K7 V2.0（Rockchip RK3576）+ WHEELTEC C50X 底盘（STM32F407VET6），Ubuntu 24.04 + ROS2 Jazzy。

---

## 核心文档

| 文件 | 内容 |
|------|------|
| `CLAUDE.md` | 项目唯一事实来源：硬件规格、资料索引、KICKPI 外设速查、STM32 底盘协议（第 13 节） |
| `AGENTS.md` | AI 助手和团队成员的工作规则 |
| `docs/K7_上手操作指南.md` | 从零上手：烧录固件、串口调试、WiFi/SSH、VSCode 远程开发 |
| `docs/K7_开发架构方案.md` | ROS2 Jazzy 架构蓝图：节点图、话题流、TF 树、分阶段路线 |

---

## 目录结构

```
K7-smart-car/
├── CLAUDE.md               # 项目主文档
├── AGENTS.md               # 工作规则
├── README.md               # 本文档
├── .gitignore
├── UPDATE_LOG.txt          # KICKPI 固件更新日志
│
├── docs/                   # 文档
│   ├── K7_上手操作指南.md
│   └── K7_开发架构方案.md
│
├── K7/                     # K7 板原理图+机械图 (V1.1/V2.0/V2.1)
├── rk3576_data/            # KICKPI 开发资料（规格书/NPU工具链/数据手册）
├── stm32_data/             # C50X 底盘资料（原理图/芯片手册/固件架构/协议头文件）
├── camera_windows_sdk/     # USB 摄像头 PC SDK（.gitignore 已排除）
│
├── STM32F407VET6_src/      # STM32 完整 Keil 工程（FreeRTOS, DIFF_CAR 两轮差速）
│
└── ROS2_src/               # ROS2 源码
    ├── wheeltec_ros2/      # 第三方参考代码（.gitignore 已排除）
    └── K7_ros2/            # ⭐ 本项目 colcon 工作区
        ├── setup_env.sh    # 一键安装 ROS2 Jazzy + 项目依赖
        ├── setup/          # GPG 密钥等安装资源
        └── src/
            ├── k7_bringup/     # C++ 串口底盘节点（含看门狗）+ EKF/IMU + udev
            ├── k7_camera/      # Python 双目 splitter + 标定
            ├── k7_description/ # URDF 机器人模型
            ├── k7_nav/         # [Phase 4] Nav2 导航参数
            └── k7_npu/         # [Phase 5] RKNN 检测节点
```

---

## 开发进度

| Phase | 内容 | 状态 |
|-------|------|------|
| 1 | K7 环境搭建（SSH/WiFi/ROS2） | ✅ Wi-Fi 已通，待装 ROS2 |
| 2 | 底盘通信（串口节点+看门狗） | ✅ 代码已写，待 K7 上编译+底盘联调 |
| 3 | 双目摄像头+深度图 | ✅ splitter 已写，待标定 |
| 4 | 自主导航（Nav2） | ⬜ 待硬件到位 |
| 5 | NPU 推理（YOLOv5） | ⬜ 待转换 .rknn 模型 |

---

## 在 K7 上构建和运行

### 1. 安装依赖

将 `ROS2_src/K7_ros2/` 拷贝到 K7 后，运行一键安装脚本：

```bash
cd ~/K7_ros2
chmod +x setup_env.sh
./setup_env.sh
```

脚本自动完成：
- 安装 ROS2 Jazzy desktop 完整版（已安装则跳过）
- 用 apt 安装本项目所有依赖包（usb_cam、joy、twist_mux、robot_localization、imu_filter_madgwick、navigation2 等）
- 将 `source /opt/ros/jazzy/setup.bash` 追加到 `~/.bashrc`

### 2. 配置 udev

```bash
cd ~/K7_ros2/src/k7_bringup/udev
chmod +x k7_udev.sh
./k7_udev.sh          # 安装规则 → 重新插拔 USB 设备
ls -l /dev/k7_controller   # 应指向 ttyUSB*
```

### 3. 编译

```bash
cd ~/K7_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`--symlink-install` 让安装目录用符号链接指向源码，Python 代码修改后无需重新 build。

### 4. 运行

```bash
source install/setup.bash
ros2 launch k7_bringup k7_core.launch.py
```

---

## 开发工作流

代码主副本在本仓库（Windows），同步到 K7 构建运行：

```bash
# Windows 端（Git Bash / PowerShell）—— 在仓库根目录：
scp -r ROS2_src/K7_ros2/ kickpi@<K7_IP>:~/

# K7 端 —— 重新构建（必要时先删 build/ install/ log/）：
cd ~/K7_ros2 && colcon build --symlink-install
```

VSCode Remote-SSH 用于板上编译调试；较大改动回本仓库提交，避免代码在 K7 和 Windows 之间漂移。

---

## 协作规范

```bash
# 第一次下载 → 拉取 → 修改 → 提交 → 推送
git clone https://github.com/V-luckyy/K7-smart-car.git 
git pull
git add .
git commit -m "feat: 描述改动"
git push
```

**Commit 格式**：`<类型>: <简述>`，类型：`docs` / `feat` / `fix` / `refactor` / `chore`

**注意事项**：
- `.img` 镜像文件、Windows 烧录工具、第三方参考代码、Keil 编译产物（`**/OBJ/`）已在 `.gitignore` 排除
- **不要提交**临时调试文件（`msg.txt`、`err_msg.txt`）
- 项目有新了解或决策变更时，同步更新 `CLAUDE.md`
- 目录结构变更时，必须同步更新所有 `.md` 文件中的路径

---

## Git 排除清单

以下文件/目录未被 Git 跟踪（详见 `.gitignore`）：

| 排除项 | 原因 |
|--------|------|
| `*.img` | 固件镜像（~4 GB） |
| `rk3576_data/5-DevelopmentTool/` | Windows 烧录工具 |
| `rk3576_data/3-SoftwareData/GCC_Cross_toolchains/` | ARM64 交叉编译器 |
| rknn-toolkit2.zip / rknpu2.zip / rknn_model_zoo.tar.gz | NPU 大文件（> 100 MB） |
| `camera_windows_sdk/` | USB 摄像头 PC SDK |
| `ROS2_src/wheeltec_ros2/` | 第三方 ROS2 参考代码 |
| `**/OBJ/` | STM32 Keil 编译产物 |

---

## 资源链接

- KICKPI：[中文](https://kickpi.cn) | [英文](https://www.kickpi.com) | [文档](https://doc.kickpi.cn)
- wheeltec ROS2 通信协议参考：`CLAUDE.md` 第 13 节
- STM32 固件参考：`stm32_data/固件架构/`
