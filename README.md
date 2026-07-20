# K7 智能小车 — RK3576 项目

> KICKPI K7 V2.0（Rockchip RK3576）+ WHEELTEC C50X 底盘（STM32F407VET6），Ubuntu 24.04 + ROS2 Jazzy。

---

## 快速开始

```bash
git clone https://github.com/V-luckyy/K7-smart-car.git
cd K7-smart-car

# 核心文档
cat CLAUDE.md                           # 硬件+架构+外设速查+STM32底盘协议
cat docs/K7_上手操作指南.md              # 烧录/串口/SSH/WiFi/VSCode

# 架构方案（团队对齐用）
cat ../../.claude/plans/buzzing-strolling-starfish.md   # 本地 plan 文件
```

---

## 项目结构

```
K7-smart-car/
├── CLAUDE.md                       # 项目主文档
├── README.md                       # 本文档
├── .gitignore                      # 排除 .img / 烧录工具 / 第三方参考代码 / 编译产物
├── docs/                           # 操作指南
│   └── K7_上手操作指南.md
├── K7/                             # K7 板原理图+机械图 (V1.1/V2.0/V2.1)
├── rk3576_data/                    # KICKPI 开发资料（详见 CLAUDE.md）
├── stm32_data/                     # C50X 底盘参考资料（原理图/手册/协议/固件源码）
├── camera windows sdk/             # USB 摄像头 PC 端 SDK（.gitignore 已排除）
│
├── ROS2_src/                       # ROS2 源码
│   ├── wheeltec_ros2/             # 参考代码（第三方，.gitignore 已排除）
│   └── K7_ros2/                   # ⭐ 本项目 colcon 工作区
│       ├── src/
│       │   ├── k7_bringup/         # 串口底盘节点(C++) + launch + ekf/imu + udev
│       │   ├── k7_camera/          # 双目 splitter(Python) + 标定
│       │   ├── k7_description/     # URDF 模型
│       │   ├── k7_nav/             # [Phase 4] Nav2 参数
│       │   └── k7_npu/             # [Phase 5] RKNN 检测节点
│       ├── setup_env.sh            # 环境脚本
│       └── README.md               # 工作区说明
│
└── update-*.img                    # 固件镜像（.gitignore 已排除）
```

---

## 开发进度

| Phase | 内容 | 状态 |
|-------|------|------|
| 1 | K7 环境搭建（SSH/WiFi/ROS2 Jazzy） | ✅ Wi-Fi 已通，待装 ROS2 |
| 2 | 底盘通信（串口节点+看门狗） | ✅ 代码已写，待 K7 上编译+底盘联调 |
| 3 | 双目摄像头+深度图 | ✅ splitter 已写，待标定 |
| 4 | 自主导航（Nav2） | ⬜ 目录已建，硬件到位后启动 |
| 5 | NPU 推理（YOLOv5） | ⬜ 待转换 .rknn 模型 |

> 详细架构方案见 plan 文件，移植清单与验证步骤均在其中。

---

## 协作规范

```bash
# 拉取最新 → 修改 → 提交 → 推送
git pull
git add .
git commit -m "feat: 描述你的改动"
git push
```

**Commit 格式**：`<类型>: <简述>`，类型：`docs` / `feat` / `fix` / `refactor` / `chore`

**注意事项**：
- `.img`、烧录工具、wheeltec 参考源码、STM32 OBJ 编译产物已在 `.gitignore` 排除
- **不要提交**临时调试文件（`msg.txt`、`err_msg.txt`）
- 每次对项目的了解或决策变更，同步更新 `CLAUDE.md`
- 开发到 K7：`scp -r ROS2_src/K7_ros2/ kickpi@<IP>:~/code/` → K7 上 `colcon build`

---

## 被排除的文件

如需使用请从 KICKPI 网盘下载或已有本地拷贝获取：

| 文件/目录 | 说明 |
|-----------|------|
| `update-*.img` | 固件镜像（~4 GB） |
| `rk3576_data/5-DevelopmentTool/` | Windows 烧录工具 |
| `rk3576_data/3-SoftwareData/GCC_Cross_toolchains/` | ARM64 交叉编译器 |
| `rk3576_data/3-SoftwareData/RKNPU/rknn-toolkit2.zip` | NPU 转换工具 |
| `rk3576_data/3-SoftwareData/RKNPU/rknpu2.zip` | NPU 运行时 |
| `rk3576_data/3-SoftwareData/Linux_rknn_yolov5/rknn_model_zoo.tar.gz` | RKNN 模型库 |
| `camera windows sdk/` | USB 摄像头 Windows SDK |
| `ROS2_src/wheeltec_ros2/` | 第三方 ROS2 参考代码 |
| `stm32_data/firmware/` | STM32 完整 Keil 工程 |

---

## 资源链接

- KICKPI: [中文](https://kickpi.cn) | [英文](https://www.kickpi.com) | [文档](https://doc.kickpi.cn)
- WHEELTEC 底盘协议：见 `CLAUDE.md` 第 13 节 和 `stm32_data/固件架构/`
- 架构方案：本地 plan 文件 `buzzing-strolling-starfish.md`
