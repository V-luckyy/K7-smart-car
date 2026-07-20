# WHEELTEC 高配版两轮差速小车 — 完整资料包结构

> **路径**: `D:\BaiduNetdiskDownload\高配版两轮差速小车附送资料`
> **内容**: 含 2 个 STM32 固件工程、硬件原理图、芯片手册、工具软件、测试脚本、文档等
> **生成日期**: 2026-07-20

---

## 顶层目录总览

```
高配版两轮差速小车附送资料/
├── WHEELTEC_C50X_2026.05.29/    [★ 主固件工程 — STM32F407+FreeRTOS]
├── WHEELTEC_C50X_VET6/           [另一固件工程变体]
├── 其他平台源码/                  [树莓派/Jetson等上位机示例代码]
├── 文档/                         [开发手册、芯片手册、电机驱动文档]
├── 原理图与资源分配图/            [各版本硬件原理图和引脚分配]
├── 32F407VET6主控芯片数据手册/    [STM32参考手册+datasheet]
├── 姿态传感器资料/                [MPU6050 / ICM20948 datasheet]
├── CAN通信驱动芯片/               [VP230 CAN收发器 datasheet]
├── 串口TTL转USB电平芯片资料/      [CH9102F datasheet]
├── 工具软件/                      [串口工具、驱动、下载器、Keil Pack]
├── 测试脚本/                      [Python测试脚本及结果图]
├── 杂项/                          [视频教程、打包资料、链接]
├── 源码工程使用指南.pdf
├── 主板详解_V1.0版.png
└── C50C_程序烧录接口串口1与串口3.jpg
```

---

## 1. WHEELTEC_C50X_2026.05.29/ — 主 STM32F4 固件工程

**这是当前开发使用的主工程。** STM32F407 (Cortex-M4F) + FreeRTOS，圆形 C50X 集成主板。

> 详细架构见 [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md)

| 项目 | 值 |
|------|-----|
| MCU | STM32F407VET6, 168MHz |
| RTOS | FreeRTOS |
| IDE | Keil MDK-ARM |
| 车型宏 | `DIFF_CAR` (当前) / AKM / MEC / 4WD / OMNI |
| 核心任务 | Balance_task(100Hz), APF_task(50Hz), data_task(20Hz) |
| 通信协议 | 11B下行控制帧, 24B上行传感器帧, CAN (0x100/0x101/0x181等) |
| APF模块 | 人工势场路径规划+避障, 三路测距传感器 |

**关键文件清单** (200+源文件, 这里只列顶层非库文件):

| 路径 | 用途 |
|------|------|
| `USER/main.c` | 入口, start_task |
| `BALANCE/balance_task.c` | 100Hz 主控制循环 (~2600行) |
| `BALANCE/apf_task.c` | 50Hz APF 路径规划 |
| `BALANCE/data_task.c` | 20Hz 数据上报 |
| `BALANCE/show_task.c` | OLED显示 + 电池管理 |
| `BALANCE/imu_task.c` | MPU6050/ICM20948 读取 |
| `BALANCE/sensor_uart.c` | 三路测距传感器UART |
| `BALANCE/uartx_callback.c` | 串口接收+协议解析 |
| `BALANCE/can_callback.c` | CAN接收 |
| `BALANCE/robot_select_init.c` | 车型选择+参数初始化 |
| `CarType/*.c` | 各车型初始化 (另一套, 当前未用) |
| `HARDWARE/` | 硬件驱动层 (~25个驱动文件) |
| `APF_MODULE/` | APF独立开发版 (与BALANCE内代码重复) |
| `(C50C)_2026-04-21.pdf` | 硬件原理图 |
| `更新记录.txt` | 版本更新记录 |
| `源码工程使用指南.pdf` | 使用说明 |

---

## 2. WHEELTEC_C50X_VET6/ — 另一固件工程变体

与 C50X_2026.05.29 同平台但可能是不同版本的固件。目录结构类似，需要对比确认差异。

**建议**: 需要比较两个工程间的差异时，diff `BALANCE/balance_task.c` 和 `BALANCE/control.c` 即可覆盖主要逻辑变更。

---

## 3. 其他平台源码/ — 上位机 & 其他平台示例

包含用于与 STM32 通信的上位机端示例代码:

| 路径 | 说明 |
|------|------|
| `2.CAN和串口控制例程发送模板/串口发送指令例程/` | STM32F103 的 CAN/串口控制示例 (Minibalance 工程), 含 MPU6050 DMP 库 |
| `.../USER/Minibalance.c` | 示例主程序 |
| `.../HARDWARE/` | 示例工程的硬件驱动 (encoder, motor, OLED, MPU6050, NRF24L01等) |

**用途**: 这些是厂家提供的参考代码, 展示了如何通过 CAN/串口向 C50X 底盘发送速度控制指令。RK3576 端串口通信可参考此处的帧格式。

---

## 4. 文档/ — 技术文档与手册

| 路径 | 内容 |
|------|------|
| `高配版两轮差速小车开发手册.pdf` | **核心文档**: 整车开发手册 |
| `项目结构与程序流程说明.md` | 项目程序流程文档 |
| `项目结构与程序流程说明.pdf` | PDF版 |
| `主控制器_C50C版/` | C50C 主板子目录 (含 datasheet、原理图、传感器资料的重制版) |
| `主控制器_C50C版/主板详解_V1.0版.png` | V1.0 主板标注图 |
| `主控制器_C50C版/主板详解_V4.0V4.1V4.2版.png` | V4.x 主板标注图 |
| `主控制器_C50C版/主板详解_V4.3版.png` | V4.3 主板标注图 |
| `主控制器_C50C版/C50C_程序烧录接口串口1与串口3.jpg` | 烧录接口实物图 |
| `主控制器_C50C版/更新记录.txt` | 版本更新记录 |
| `3.原理图/` | STM32F103RC 控制器原理图、BTN驱动模块原理图、转接板原理图 |
| `电机驱动/` | D50A 双通道直流有刷电机驱动器用户手册 (新旧两版) |
| `芯片手册/` | BTN7971, MPU6050, STM32F103, VP230 等 datasheet |
| `STM32F103技术文档/` | Cortex-M3权威指南、STM32参考手册 |
| `主控制器_C50C版/32F407VET6主控芯片数据手册/` | (与顶层重复) |
| `主控制器_C50C版/姿态传感器资料/` | (与顶层重复) |
| `主控制器_C50C版/原理图与资源分配图/` | (与顶层重复) |
| `主控制器_C50C版/CAN通信驱动芯片/` | (与顶层重复) |
| `主控制器_C50C版/串口TTL转USB电平芯片资料/` | CH9102F datasheet |

---

## 5. 原理图与资源分配图/ — 硬件设计资料

按主板硬件版本组织:

| 路径 | 内容 |
|------|------|
| `说明.txt` | 版本说明 |
| `V1.0版/C50C主控原理图.pdf` | V1.0 原理图 |
| `V1.0版/C50C集成板资源分配.pdf` | V1.0 引脚分配 |
| `V1.0版/C50C集成板资源分配表.pdf` | V1.0 引脚分配表 |
| `V4.0 4.1 4.2版/C50C-V4.0原理图.pdf` | V4.0 原理图 |
| `V4.0 4.1 4.2版/C50C-V4.1V4.2原理图.pdf` | V4.1/V4.2 原理图 |
| `V4.0 4.1 4.2版/C50C-V4.0V4.1V4.2集成板资源分配.pdf` | V4.x 引脚分配 |
| `V4.3版/C50C_V4.3原理图.pdf` | V4.3 原理图 |
| `V4.3版/C50C-V4.3集成板资源分配.pdf` | V4.3 引脚分配 |

**用途**: 查找任意引脚的硬件连接关系。固件中通过 `SysVal.HardWare_Ver` 区分不同硬件版本。

---

## 6. 芯片数据手册

| 路径 | 芯片 | 用途 |
|------|------|------|
| `32F407VET6主控芯片数据手册/STM32F407VET6.PDF` | STM32F407VET6 | 主 MCU |
| `32F407VET6主控芯片数据手册/STM32中文参考手册_V10.pdf` | STM32F4 系列 | 编程参考 |
| `姿态传感器资料/V1.0版/MPU6050原版英文手册.PDF` | MPU6050 | 6轴IMU (V1.0硬件) |
| `姿态传感器资料/V4.0及以上版本/ICM-20948.pdf` | ICM-20948 | 9轴IMU (V4.0+硬件) |
| `CAN通信驱动芯片/VP230.pdf` | VP230 | CAN收发器 |
| `串口TTL转USB电平芯片资料/` | CH9102F | USB-UART芯片 |

**文档/芯片手册/** 中包含额外副本: BTN7971(电机驱动), MPU6050, STM32F103 等。

---

## 7. 工具软件/

| 路径 | 用途 |
|------|------|
| `软件与驱动/串口调试助手（丁丁）/sscom33.exe` | 串口调试工具 |
| `软件与驱动/CH340驱动(USB串口驱动)_XP_WIN7_WIN8共用/` | USB串口驱动 |
| `软件与驱动/STM32F4串口下载软件（FLYMCU）/FlyMcu.exe` | STM32 ISP串口下载 |
| `软件与驱动/mcuisp(用于STM32串口下载程序).exe` | 另一个串口下载工具 |
| `Keil.STM32F1xx_DFP.2.4.1.pack` | Keil STM32F1 器件包 |
| `Keil.STM32F4xx_DFP.3.1.1.pack` | Keil STM32F4 器件包 |

---

## 8. 测试脚本/

| 路径 | 用途 |
|------|------|
| `test_kalman.py` | 卡尔曼滤波测试脚本 |
| `test_sensor_fusion.py` | 传感器融合测试脚本 |
| `kalman_test_result.png` | 卡尔曼测试结果图 |
| `sensor_fusion_test.png` | 传感器融合测试结果图 |

**用途**: 这两个 Python 脚本是 APF 模块中卡尔曼滤波和传感器融合算法的验证代码。在 STM32 上实现前, 先用 Python 验证算法正确性。

---

## 9. 杂项/

| 路径 | 用途 |
|------|------|
| `高配版两轮差速小车测试视频教程.mp4` | 测试视频教程 |
| `两轮差速小车资料.zip` | 资料打包 |
| `网盘链接.txt` | 网盘链接 |

---

## 10. 文件去重提醒

以下内容在资料包中**存在多个副本**, 开发时注意以哪个为准:

| 内容 | 副本位置 |
|------|----------|
| STM32F407 datasheet | `32F407VET6主控芯片数据手册/` 和 `文档/主控制器_C50C版/32F407VET6主控芯片数据手册/` |
| MPU6050 datasheet | `姿态传感器资料/` 和 `文档/芯片手册/` 和 `文档/主控制器_C50C版/姿态传感器资料/` |
| ICM20948 datasheet | `姿态传感器资料/` 和 `文档/主控制器_C50C版/姿态传感器资料/` |
| VP230 datasheet | `CAN通信驱动芯片/` 和 `文档/主控制器_C50C版/CAN通信驱动芯片/` |
| C50C 原理图 | `原理图与资源分配图/` 和 `文档/主控制器_C50C版/原理图与资源分配图/` |
| APF 源码 | `WHEELTEC_C50X_2026.05.29/BALANCE/` 和 `WHEELTEC_C50X_2026.05.29/APF_MODULE/` |
| 机器人参数初始化 | `WHEELTEC_C50X_2026.05.29/BALANCE/robot_select_init.c` 和 `WHEELTEC_C50X_2026.05.29/CarType/diff_robot_init.c` |

---

## 11. 固件工程间关系

```
                              WHEELTEC C50X 固件生态
                                       |
                 +---------------------+---------------------+
                 |                                           |
    WHEELTEC_C50X_2026.05.29/                   WHEELTEC_C50X_VET6/
    (最新主工程, 当前开发使用)                    (另一版本工程)
    STM32F407 + FreeRTOS                        STM32F407 + FreeRTOS
    含 APF 模块                                  结构类似, 需对比
    5种车型支持
    多模式控制
                 |
    通信协议 (11B下行 / 24B上行 / CAN)
                 |
                 v
    其他平台源码/ 中的示例代码
    (STM32F103 Minibalance 工程)
    展示了如何通过串口/CAN 向底盘发速度指令
```

---

## 12. Agent 工作指南

其他 agent 读取本文档后可以快速定位需要的资源:

| 需求 | 查看 |
|------|------|
| 了解固件架构 | `WHEELTEC_C50X_2026.05.29/docs/PROJECT_ARCHITECTURE.md` |
| 修改运动控制逻辑 | `BALANCE/balance_task.c` (100Hz主循环) |
| 修改 APF 避障参数 | `BALANCE/Inc/apf.h` (宏定义) 和 `BALANCE/apf_task.c` |
| 修改通信协议 | `BALANCE/data_task.c` (上行), `BALANCE/uartx_callback.c` (下行) |
| 添加新传感器 | 参考 `BALANCE/sensor_uart.c` 的模式, 在 `sensor_uart.h` 中声明全局变量 |
| 查找引脚映射 | `原理图与资源分配图/` 对应硬件版本 |
| 查找芯片寄存器 | `32F407VET6主控芯片数据手册/` 和 `姿态传感器资料/` |
| 理解通信协议格式 | `BALANCE/Inc/data_task.h` (结构体定义) |
| RK3576 端串口通信参考 | `其他平台源码/2.CAN和串口控制例程发送模板/` |
| Python算法验证 | `测试脚本/test_sensor_fusion.py`, `test_kalman.py` |
| 烧录工具 | `工具软件/软件与驱动/STM32F4串口下载软件（FLYMCU）/` |

---

> **最后更新**: 2026-07-20
> **配套文档**: [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) — C50X_2026.05.29 工程详细架构
