# 项目指导文件

本项目的核心文档为根目录下的 `CLAUDE.md`（K7 智能小车 — RK3576 项目文档），`docs/` 下还有上手指南和架构方案。

**每次开始工作前，必须先阅读 `CLAUDE.md`**，并严格遵循其中约定。

---

## 项目目录

```
K7-smart-car/
├── CLAUDE.md                   # ⭐ 项目主文档（唯一事实来源）
├── AGENTS.md                   # 本文档
├── README.md                   # GitHub 首页
├── .gitignore                  # 排除 .img / 烧录工具 / 第三方参考 / 编译产物
│
├── docs/                       # 文档
│   ├── K7_上手操作指南.md       # 烧录/SSH/WiFi/VSCode
│   └── K7_开发架构方案.md       # ROS2 架构蓝图（终稿）
│
├── K7/                         # K7 板原理图+机械图
├── rk3576_data/                # KICKPI 开发资料（规格书/数据手册/NPU工具链）
├── stm32_data/                 # C50X 底盘资料（原理图/手册/固件架构文档/协议头文件）
├── camera_windows_sdk/         # USB 摄像头 PC SDK（.gitignore 已排除）
├── STM32F407VET6_src/          # STM32 完整 Keil 工程源码（两轮差速，DIFF_CAR target）
│
└── ROS2_src/                   # ROS2 源码
    ├── wheeltec_ros2/          # 参考代码（第三方，.gitignore 已排除）
    └── K7_ros2/                # ⭐ 本项目 colcon 工作区
        ├── src/
        │   ├── k7_bringup/     # 串口底盘节点(C++) + launch + ekf/imu + udev
        │   ├── k7_camera/      # 双目 splitter(Python) + 标定
        │   ├── k7_description/ # URDF 模型
        │   ├── k7_nav/         # [Phase 4] Nav2 参数
        │   └── k7_npu/         # [Phase 5] RKNN 检测节点
        └── README.md
```

## 关键事实

- 硬件：K7 V2.0 + C50X 底盘 V1.0（STM32F407，两轮差速）
- 通信：USB 转串口（CH9102F），115200 8N1，协议与 wheeltec 逐字节一致
- ROS2 版本：**Jazzy**
- 供电：双电池独立（K7 插电自启，底盘按键开机），USB 线共地
- ⚠️ 固件断链不停车 → ROS 端必须实现 cmd_vel 看门狗

## 规则

1. **每次对该项目有新的了解、决策或变更，必须更新 `CLAUDE.md`**
2. 大文件/第三方代码已通过 `.gitignore` 排除，添加新文件前确认是否会误提交
3. 开发工作流：Windows 写码 → `scp` 到 K7 → `colcon build`
4. 提交格式：`<类型>: <简述>`（docs/feat/fix/refactor/chore）
