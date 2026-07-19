# K7 智能小车主板 — RK3576 项目文档

> **最后更新**: 2026-07-19（添加摄像头 Windows SDK + 完善资料索引）
> **板卡型号**: KICKPI K7 V2.0
> **主控芯片**: Rockchip RK3576
> **项目用途**: 智能小车主控 — 运行 Ubuntu + ROS2，负责路径规划、深度图计算、上位机通信、底层 STM32 驱动
> **本地资料**: `rk3576_data/` 已清理，保留 K7 相关 + NPU 工具链 + 开发工具，移除 K7C/K7S/Android 等无关内容

---

## 1. 硬件规格

### 主控 SoC — RK3576

| 项目 | 参数 |
|------|------|
| **CPU** | 8 核：4×Cortex-A72 + 4×Cortex-A53，最高 2.0 GHz |
| **制程** | 22nm |
| **GPU** | ARM Mali G52 MC3（OpenGL ES 3.2 / Vulkan 1.2 / OpenCL 2.2） |
| **NPU** | 6 TOPS，支持 INT4/INT8/INT16/FP16（RKNN 推理框架） |
| **ISP** | 板载高性能 ISP，支持多路摄像头图像处理 |

### 内存与存储

| 项目 | 参数 |
|------|------|
| **RAM** | LPDDR4X，4GB / 8GB / 16GB 可选 |
| **板载存储** | eMMC 16GB / 32GB / 64GB |
| **扩展存储** | MicroSD 卡槽、M.2 M-Key（NVMe SSD）、SATA 3.0 |

### 网络

| 项目 | 参数 |
|------|------|
| **以太网** | 双千兆 RJ45（1000M ×2） |
| **WiFi / BT** | RTL8822CS（2T2R，WiFi 5 双频，Bluetooth） |
| **4G/5G** | Mini PCIe 接口，支持 EC20/EC200 4G、RG200U 5G 模块 |

### USB

| 项目 | 参数 |
|------|------|
| **USB 3.0 Host** | ×3 |
| **USB Type-C** | USB 3.1 OTG / Debug / DP v1.4 视频输出 |

### 显示接口

| 接口 | 最大分辨率 |
|------|-----------|
| **HDMI** | 4K @ 60Hz |
| **eDP** | 2560×1600 @ 60Hz |
| **MIPI DSI** ×2 | 1920×1080 @ 60Hz |
| **LVDS** | 1920×1080 @ 60Hz |
| **Type-C DP v1.4** | 4K |

> 支持三屏异显，最大四屏独立显示。

---

## 2. 摄像头接口（双目立体视觉）

这是本项目的关键接口，用于接收双目摄像头数据进行深度图计算。

### MIPI CSI 规格

| 项目 | 参数 |
|------|------|
| **接口数量** | 1× 4-Lane MIPI CSI 或 2× 2-Lane MIPI CSI |
| **支持的摄像头传感器** | IMX415（官方已适配）、MAE0621A |
| **FPC 连接器** | 20-Pin 摄像头排线座 |

### 双目摄像头方案

K7 的 MIPI CSI 可以拆分为 **2×2-Lane** 模式，接入两个摄像头作为双目立体视觉输入。这对于：

- **ROS2 深度图计算**（stereo matching / disparity map）
- **视觉 SLAM**
- **目标检测与避障**

### 已适配的摄像头驱动

- **IMX415** — 通过 V2.0 固件添加了 IMX415 配置支持（见 UPDATE_LOG 2025-10-20）
- ISP 支持 3A（AE/AWB/AF）、HDR、降噪等

---

## 3. 40-Pin GPIO 扩展口

与底层 STM32 通信的主要接口，可用于 UART/SPI/CAN 等通信方式。

| 功能 | 可用数量 |
|------|---------|
| **UART** | ×5 |
| **I2C** | ×3 |
| **SPI** | ×1 |
| **PWM** | ×7 |
| **ADC** | ×3 |
| **I3C** | ×1 |
| **CAN / CAN-FD** | ×2 |
| **GPIO** | ×22 |
| **PDM** | ×1 |

### 与 STM32 通信方案（待确认）

推荐方案（按优先级）：
1. **UART（串口）** — 最简单可靠，双向通信
2. **CAN / CAN-FD** — 工业级抗干扰好，适合电机控制
3. **SPI** — 高速但占线多

---

## 4. 操作系统

| 系统 | 版本 | 支持状态 |
|------|------|---------|
| **Ubuntu** | 24.04 | ✅ 官方支持（本项目选用） |
| **Debian** | 12 | ✅ 官方支持 |
| **Android** | 14 | ✅ 官方支持 |
| **Armbian** | Noble (24.04) | ✅ 社区支持 |

### 镜像类型对比

| | 标准 Ubuntu 24.04 | Armbian Ubuntu 24.04 |
|---|---|---|
| **桌面** | XFCE（轻量） | GNOME |
| **烧录方式** | RKDevTool → 升级固件 → 一键升级 | RKDevTool → 下载镜像 → 加载 cfg → 手动分区 |
| **MIPI 屏幕** | 自动识别 | 手动编辑 `/boot/armbianEnv.txt` |
| **SD 卡工具** | SDDiskTool | balenaEtcher |
| **维护方** | KICKPI 官方 | 社区（Armbian 源码） |
| **适合本项目** | ✅ 推荐 | ❌ 不需要 |

> **本项目选用标准 Ubuntu 24.04 镜像**，文件名格式：`update-rk3576-kickpi-k7-linux-ubuntu-multi-YYYYMMDD-HHMMSS.img`

### 默认登录凭据

| 系统 | 用户名 | 密码 |
|------|--------|------|
| **Ubuntu 24.04** | `kickpi` | `kickpi` |
| Ubuntu 24.04 | `root` | `root` |
| Debian 12 | `linaro` | `linaro` |
| Armbian | `kickpi` | `kickpi` |

### 无屏幕操作（Headless）

K7 完全支持无屏幕工作，三种方式获取板子 IP：

| 方式 | 说明 |
|------|------|
| **串口调试线** | USB 转 TTL 连接 DEBUG 口，MobaXterm 波特率 **1500000**，登录后 `ifconfig` |
| **路由器后台** | 查看 DHCP 分配列表 |
| **HDMI 临时接一下** | 登录后 `ifconfig eth0` |

拿到 IP 后通过 SSH 连接（默认已开启）：
```bash
ssh kickpi@<IP>
# 密码: kickpi
```

如需 root 通过 SSH 登录，需在板子上编辑：
```bash
sudo sed -i 's/#PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config
sudo systemctl restart sshd
```

### 串口调试接线

K7 DEBUG 口使用 USB 转 TTL 模块：

| TTL 线 | 接 K7 |
|--------|-------|
| GND（黑） | GND |
| TX（绿） | RX |
| RX（白） | TX |
| VCC（红） | **不接** |

> 如无输出，交换 TX/RX 再试。MobaXterm 设置：Session → Serial → 选 COM 口 → Speed = **1500000**。
- 支持 SD 卡启动引导（2025-10-20 添加）

### 固件烧录指南

> 来源: https://doc.kickpi.cn/Products/Rockchip-Image-Installing/RK3576-Image-Installation/

#### 前置准备

- Windows PC + **可传数据的 Type-C 线**（USB-A to C 比 C-to-C 更可靠，廉价充电线可能无法传数据）
- 板子供电电源
- HDMI 显示器（可选，监控烧录进度）
- TF/SD 卡（≥16GB，如需 SD 卡方式）
- **先装 USB 驱动**：`rk3576_data/5-DevelopmentTool/win_x64_UsbDriver/DriverAssitant_v5.13.zip` → 解压运行 `DriverInstall.exe`

#### 进入 Loader / Maskrom 模式

K7 板上有 **MASKROM/RECOVERY**、**RESET** 两个按键：

| 方式 | 操作 |
|------|------|
| **断电上电法** | 板子断电 → 按住 RECOVERY → 接通电源+USB → RKDevTool 检测到设备后松手 |
| **复位法** | 板子已通电+USB 已连 → 按住 RECOVERY → 短按 RESET → 松手 |

> LOADER 模式用于正常刷机，MASKROM 模式用于救砖强制烧录。同一个按键触发，检测到后 RKDevTool 状态栏会显示 "Found One LOADER/MASKROM Device"。

#### 方式一：USB 直接烧录到 eMMC（推荐）

适用 Ubuntu / Debian / Android 三类镜像。

1. 解压并运行 `RKDevTool_v3.30_for_window.zip` 中的 `RKDevTool.exe`
2. 板子进入 Loader 模式，连接 PC，确认 RKDevTool 检测到设备
3. 切换到 **"升级固件"（Upgrade Firmware）** 标签页
4. 点击 **"固件"（Firmware）**，选择 `update-*.img` 镜像文件
5. 点击 **"升级"（Upgrade）**，等待完成

**烧录完成后**：
- 有显示器：自动重启进桌面
- 无显示器：绿色 LED 常亮 + 蓝色 LED 持续闪烁 = 系统就绪

#### 方式二：SD 启动卡（便携系统）

适合测试、维护、eMMC 故障时使用。K7 启动优先级：**SD 卡 > eMMC**。

⚠️ **如果 eMMC 里有 Android 而要 SD 启动 Linux**，eMMC 的旧 U-boot 会冲突，需要先擦除 eMMC（RKDevTool → 高级功能 → EraseAll）。

1. 解压 `SDDiskTool_v1.78.zip`，运行 `SD_Firmware_Tool.exe`
2. SD 卡插入 PC，工具中选择该卡
3. 选择 **"SD 启动卡"** 模式
4. 点击 **固件** 选择 `update-*.img`
5. 点击 **"创建"** 等待完成（如格式化失败→见下方故障排除）
6. 板子断电 → 插入 SD 卡 → 上电启动
7. **首次启动**可能在 ROCKCHIP KERNEL logo 卡 5-10 分钟，屏幕反复黑屏是正常的，等待即可

#### 方式三：SD 安装卡（写入 eMMC）

SD 卡作为安装介质，**会覆盖 eMMC**。

1. SDDiskTool 中选择 **"SD 安装卡"** 模式
2. 卡容量 >16GB 时需编辑 `SDDiskTool_v1.74/config.ini`，设 `USER_DISK_FS=NTFS`
3. 创建后插入板子，上电启动，屏幕显示进度条
4. 完成后显示 **"Please remove SD CARD!!!, wait for reboot"** → 拔卡自动重启
5. 无显示器时：等蓝色和绿色 LED 都常亮后拔卡

#### 故障排除

| 问题 | 解决 |
|------|------|
| **SDDiskTool 格式化失败** | 编辑 `config.ini`，`USER_DISK_FS=NTFS` |
| **Armbian 烧录失败后无法启动** | 先刷完整的 Android/Linux 镜像恢复，再重试 Armbian |
| **SD 卡启动冲突** | RKDevTool → 高级功能 → EraseAll 擦除 eMMC |
| **USB 连接不上** | 换一根可传数据的 Type-C 线（USB-A to C 优先） |

---

## 5. 软件架构（计划）

```
┌─────────────────────────────────────────────┐
│                 上位机 / PC                  │
│         (视功能需求，可能运行 Rviz 等)         │
└─────────────────┬───────────────────────────┘
                  │ WiFi / Ethernet / 4G/5G
┌─────────────────┴───────────────────────────┐
│              K7 主板 (RK3576)                │
│                                              │
│  ┌──────────────────────────────────────┐    │
│  │         Ubuntu 24.04                  │    │
│  │  ┌────────────────────────────────┐  │    │
│  │  │  ROS2 (Humble / Jazzy)         │  │    │
│  │  │  - 路径规划 (Nav2)              │  │    │
│  │  │  - 深度图计算 (Stereo Matching) │  │    │
│  │  │  - 传感器融合                   │  │    │
│  │  └────────────────────────────────┘  │    │
│  │  ┌────────────────────────────────┐  │    │
│  │  │  RKNN (6 TOPS NPU)             │  │    │
│  │  │  - 目标检测 / 车道检测          │  │    │
│  │  │  - 视觉 SLAM 加速               │  │    │
│  │  └────────────────────────────────┘  │    │
│  │  MIPI CSI ── 双目摄像头输入          │    │
│  │  UART/CAN ── STM32 底层通信          │    │
│  └──────────────────────────────────────┘    │
└─────────────────┬───────────────────────────┘
                  │ UART / CAN
┌─────────────────┴───────────────────────────┐
│             STM32 底层驱动板                  │
│         - 电机控制 (PID)                      │
│         - 编码器读取                           │
│         - IMU 传感器读取                       │
│         - 电池电压监测                         │
└─────────────────────────────────────────────┘
```

---

## 6. 本地资料目录结构

```
D:\BaiduNetdiskDownload\RK3576\
├── CLAUDE.md                                   ← 本文档
├── UPDATE_LOG.txt                               ← 固件更新日志
├── K7\                                          ← 原始资料
│   ├── 原理图\  (V1.1 / V2.0 / V2.1)
│   └── 机械图\  (PCB尺寸图 / DXF)
│
└── rk3576_data\                                 ← 从 kickpi 网盘下载的资料（已清理无关内容）
    ├── 0-Specifications\
    │   └── K7-RK3576\
    │       ├── KICKPI-K7_Specification.pdf       ← K7 英文规格书
    │       └── KICKPI-K7规格书.pdf                ← K7 中文规格书
    │
    ├── 3-SoftwareData\
    │   ├── GCC_Cross_toolchains\
    │   │   └── gcc-arm-10.3-...-aarch64-...tar.xz ← ARM64 交叉编译工具链
    │   ├── Linux_Pack_Firmware\
    │   │   └── Linux_Pack_Firmware_RK3576.zip     ← Linux 固件打包工具
    │   ├── Linux_backup_rootfs_script\
    │   │   ├── armbian_replace_rootfs.sh          ← 替换 rootfs 脚本
    │   │   └── ff_export_rootfs                   ← 导出 rootfs 工具
    │   ├── Linux_rknn_yolov5\
    │   │   ├── rknn_yolov5_demo_Linux_rk3576.zip  ← YOLOv5 NPU 推理 demo
    │   │   ├── rknn_model_zoo.tar.gz              ← RKNN 模型库
    │   │   └── yolov5_test.h264                   ← 测试视频
    │   ├── Linux_spi_tool\
    │   │   └── spidev_test                        ← SPI 通信测试工具
    │   └── RKNPU\
    │       ├── NPU-DOCS.zip                        ← NPU 开发文档
    │       ├── rknn-toolkit2.zip                   ← 模型转换工具 (PC端)
    │       └── rknpu2.zip                          ← NPU 运行时库 (板端)
    │
    ├── 4-HardwareData\
    │   ├── datasheet\
    │   │   ├── RK3576_Brief_Datasheet_V1.3.pdf    ← 芯片简介
    │   │   ├── Rockchip_RK3576_Datasheet_V1.1.pdf ← 完整数据手册
    │   │   ├── Rockchip_RK3576_TRM_Part1_V1.2.pdf ← 技术参考手册 (上)
    │   │   └── Rockchip_RK3576_TRM_Part2_V1.2.pdf ← 技术参考手册 (下)
    │   └── K7\
    │       ├── 原理图\  (V1.1 / V2.0 / V2.1)
    │       ├── 机械图\  (PCB尺寸图 / DXF)
    │       └── PIN脚定义参考-K7S\                  ← K7S GPIO 映射表（同芯片不同板型，可参考复用关系）
    │           ├── K7S-PIN引脚定义-功能映射表.xlsx
    │           └── PIN.png
    │
    └── 5-DevelopmentTool\                         ← Windows 开发工具
        ├── win_x64_UsbDriver\                     ← USB 驱动 (烧录必装)
        ├── win_x64_UsbImageBurnTool\               ← RKDevTool 主烧录工具
        ├── win_x64_SDDiskTool\                     ← SD 卡烧录工具
        ├── win_x64_FactoryTool\                    ← 量产烧录工具
        ├── win_x64_UartDebugTool\                  ← MobaXterm 串口调试
        ├── win_x64_QtScrcpy\                       ← 屏幕投屏工具
        └── win_x64_RKDevInfoWriteTool\             ← MAC/SN 写入工具
│
└── camera windows sdk\                         ← USB 摄像头 PC 端 SDK
    ├── TXY_Win-SDK开发文档.docx                    ← 开发文档
    └── USBCamera_SDK/DirectShow/                  ← DirectShow SDK（含头文件、库、示例）
```

---

## 7. USB 摄像头 Windows SDK

> `camera windows sdk/` — 用于在 Windows PC 端开发和测试 USB 摄像头的接入代码，运行在 x86/x64 Windows 上，不在 K7 上使用。

### SDK 概览

| 项目 | 说明 |
|------|------|
| **厂商** | TXY（图新源？） |
| **SDK 类型** | Microsoft DirectShow |
| **总大小** | ~267 MB |
| **适用范围** | Windows（x86 / x64） |
| **在项目中的作用** | PC 端验证摄像头参数、调试图像质量、开发图像采集逻辑，确认无误后移植到 K7 (Linux/V4L2) |

### SDK 目录结构

```
camera windows sdk/
├── TXY_Win-SDK开发文档.docx           ← 摄像头 SDK 开发文档（Word）
└── USBCamera_SDK/
    └── DirectShow/
        ├── Include/                  ← C/C++ 头文件（DirectShow API）
        ├── Lib/
        │   ├── x64/                  ← 64 位静态库
        │   └── x86/                  ← 32 位静态库
        ├── Samples/
        │   ├── C++/DirectShow/       ← C++ 示例（BaseClasses / Filters / Players）
        │   └── VB Samples/           ← VB 示例（Builder / Editing / Player / WebDVD）
        ├── Documentation/            ← DirectShow API 文档（.chm）
        └── Utilities/                ← 开发工具（GraphEdit 等）
```

### 核心库文件（Lib/）

| 库文件 | 用途 |
|--------|------|
| `strmiids.lib` / `amstrmid.lib` | DirectShow 核心接口 |
| `quartz.lib` | Filter Graph Manager |
| `dmoguids.lib` / `msdmo.lib` | DMO (DirectX Media Objects) |
| `ksproxy.lib` / `ksuser.lib` | 内核流（Kernel Streaming）代理 |

### 与 K7 的关系

| 阶段 | 平台 | 说明 |
|------|------|------|
| **PC 端开发/调试** | Windows | 用此 SDK 在 VS 中运行摄像头，测试分辨率、帧率、曝光等参数 |
| **K7 端运行** | Linux (Ubuntu 24.04) | 不使用此 SDK，使用 **V4L2 + OpenCV** 或 **GStreamer** 捕获摄像头 |
| **移植流程** | — | PC 端验证参数 → 确认摄像头型号 → 在 K7 上用 V4L2 打开同款摄像头 |

> **注意**：此 SDK 为 **Windows DirectShow** 专用，不能直接在 K7 (ARM64 Linux) 上编译或运行。它是 PC 端的开发辅助工具。

---

## 8. 资源链接

### 官网

- 官网（中文）: https://kickpi.cn
- 官网（英文）: https://www.kickpi.com
- 文档/下载: https://doc.kickpi.cn → 产品 → K7

### 本地已有资料（无需再从网络获取）

- ✅ K7 原理图（V1.1 / V2.0 / V2.1）
- ✅ K7 机械图（PCB 尺寸 / DXF）
- ✅ K7 规格书（中英文）
- ✅ RK3576 数据手册 + TRM Part1 & Part2
- ✅ RKNN NPU 开发文档 + 模型转换工具 + 运行时
- ✅ YOLOv5 Linux demo（含模型库和测试视频）
- ✅ ARM64 交叉编译工具链 (GCC 10.3)
- ✅ Windows 烧录/调试全套工具
- ✅ USB 摄像头 Windows SDK（DirectShow，含文档和示例）
- ⚠️ Rockchip SDK 暂不开放，定制需联系 KICKPI 技术支持

---

## 9. 待确认 / 待完善事项

### 硬件相关
- [x] **K7 板的具体版本** → V2.0，对应原理图 `K7_V2.0_20250716_SCH.pdf`
- [ ] **MIPI CSI 20-Pin FPC 线序定义** → 查本地原理图 `K7_V2.0_20250716_SCH.pdf` 的摄像头部分
- [ ] **双目摄像头型号**（IMX415 已适配 V2.0，但需确认左右摄像头如何接入 — 1×4-Lane 还是 2×2-Lane？）
- [ ] **供电方案**（电池电压？12V 输入？DC 口还是 40Pin 供电？）
- [ ] **M.2 NVMe SSD 是否需要**（日志/地图数据存储？）

### 通信相关
- [ ] **与 STM32 的通信协议** — UART 串口（简单）还是 CAN 总线（抗干扰）？波特率？数据帧格式？
- [ ] **STM32 板的具体型号、接口定义、通信协议文档**
- [ ] **上位机通信方式** — WiFi / 以太网 / 4G/5G？MAVLink 协议 or 自定义 ROS2 topic？
- [ ] **双以太网口的用途分配** — 哪个接上位机？哪个留作调试？

### 传感器相关
- [ ] **IMU 型号及安装位置**（板载？STM32 外挂？型号如 MPU6050/ICM20948？）
- [ ] **是否需要其他传感器**（超声波、ToF、激光雷达？）

### 软件相关
- [ ] **ROS2 版本** — Humble（Ubuntu 22.04） vs Jazzy（Ubuntu 24.04）。板子官方支持 Ubuntu 24.04，建议 Jazzy
- [ ] **NPU 推理方案** — YOLOv5 demo 已就绪，确认具体检测目标（行人/车道线/障碍物？）
- [ ] **深度图算法** — Stereo SGBM？还是基于学习的深度估计？
- [ ] **启动方式** — eMMC 直接启动 or SD 卡启动？

---

## 10. 关键驱动和软件依赖

### 基础开发环境

| 工具 | 路径 | 用途 |
|------|------|------|
| **ARM64 交叉编译器** | `rk3576_data/3-SoftwareData/GCC_Cross_toolchains/` | 在 x86 PC 上编译 ARM64 程序 |
| **RKDevTool** | `rk3576_data/5-DevelopmentTool/win_x64_UsbImageBurnTool/` | 固件烧录（主工具） |
| **SDDiskTool** | `rk3576_data/5-DevelopmentTool/win_x64_SDDiskTool/` | SD 卡烧录 |
| **MobaXterm** | `rk3576_data/5-DevelopmentTool/win_x64_UartDebugTool/` | 串口调试 |
| **USB Driver** | `rk3576_data/5-DevelopmentTool/win_x64_UsbDriver/` | 烧录驱动（必须先装）|

### NPU 工具链

| 组件 | 路径 | 用途 |
|------|------|------|
| **NPU-DOCS.zip** | `rk3576_data/3-SoftwareData/RKNPU/` | NPU API 文档、算子支持列表 |
| **rknn-toolkit2.zip** | `rk3576_data/3-SoftwareData/RKNPU/` | PC 端模型转换（PyTorch/ONNX → RKNN）|
| **rknpu2.zip** | `rk3576_data/3-SoftwareData/RKNPU/` | 板端运行时库（C API） |
| **YOLOv5 Demo** | `rk3576_data/3-SoftwareData/Linux_rknn_yolov5/` | 已适配 RK3576 的 Linux NPU demo |

### 系统驱动

- **MIPI CSI 摄像头驱动** — IMX415 / MAE0621A，V4L2 框架（固件已集成）
- **ISP 驱动** — librkisp / libgstreamer 插件
- **GPU 驱动** — Mali G52（OpenGL ES / OpenCL / Vulkan）
- **CH341 驱动** — USB 转串口（固件已集成）
- **TUN 驱动** — VPN/Tunnel（固件已集成）

### ROS2 相关软件包

- `ros-jazzy-desktop`（推荐，匹配 Ubuntu 24.04）
- `nav2`（Navigation2 路径规划）
- `stereo_image_proc`（双目深度图计算）
- `cv_bridge`（ROS ↔ OpenCV 图像桥接）
- `robot_localization`（传感器融合）
- `image_transport`（图像传输优化）

---

> **提示**：每次对该项目有新的了解、决策或变更时，请更新此文档。尤其是待确认事项部分，确认一项就划掉一项。
