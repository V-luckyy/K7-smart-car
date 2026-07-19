# K7 智能小车主板 — RK3576 项目

> 基于 KICKPI K7 V2.0（Rockchip RK3576）的智能小车项目，运行 Ubuntu 24.04 + ROS2，负责路径规划、深度图计算、上位机通信与底层 STM32 驱动。

---

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/V-luckyy/K7-smart-car.git
cd K7-smart-car

# 2. 阅读项目文档
# 项目整体说明（硬件规格、软件架构、待办事项）
cat CLAUDE.md

# 上手操作指南（烧录、串口、网络、SSH）
cat docs/K7_上手操作指南.md
```

---

## 目录结构

```
K7-smart-car/
├── CLAUDE.md                       # 项目主文档（硬件、架构、待办清单）
├── UPDATE_LOG.txt                  # KICKPI 固件更新日志
├── docs/                          # 操作指南等文档
│   └── K7_上手操作指南.md
├── K7/                            # 原理图 + 机械图
│   ├── 原理图/
│   └── 机械图/
└── rk3576_data/                   # 开发资料
    ├── 0-Specifications/          # K7 规格书（中英文）
    ├── 3-SoftwareData/            # NPU 工具、Demo、脚本
    │   ├── RKNPU/                 # NPU 文档（大文件 zip 已排除）
    │   ├── Linux_rknn_yolov5/     # YOLOv5 Demo（大文件已排除）
    │   ├── Linux_Pack_Firmware/   # 固件打包工具
    │   ├── Linux_backup_rootfs_script/  # rootfs 备份脚本
    │   └── Linux_spi_tool/        # SPI 通信测试工具
    └── 4-HardwareData/            # 数据手册、TRM、引脚定义
        ├── datasheet/             # RK3576 Datasheet + TRM Part1/2
        └── K7/                    # 原理图、机械图、GPIO 映射表
```

---

## 协作规范

### 基本流程

```bash
# 1. 开始工作前，拉取最新代码
git pull

# 2. 修改文件后，提交并推送
git add .                          # 或指定具体文件
git commit -m "简要描述你的改动"
git push

# 3. 推送前如果别人已经更新了，先拉取再推送
git pull && git push
```

### Commit 信息规范

```
<类型>: <简述>

类型：docs（文档）| feat（新功能）| fix（修复）| refactor（重构）| chore（杂项）
```

示例：
```
docs: 更新上手操作指南 — MASKROM 模式说明
feat: 添加 WiFi 自动连接配置脚本
```

### 注意事项

- **不要提交大文件**（镜像 `.img`、烧录工具等），已在 `.gitignore` 中排除
- **不要提交临时文件**（`msg.txt`、`err_msg.txt` 等调试输出）
- **更新 CLAUDE.md**：每次对项目有新的了解或决策，同步更新该文档
- Push 前检查 `git status`，确认没有误提交不需要的文件

---

## 被排除的文件

以下文件未纳入 Git 版本管理（太大或可单独获取），如需使用请从 KICKPI 网盘下载或向已有本地拷贝的成员获取：

| 文件/目录 | 说明 |
|-----------|------|
| `update-*.img` | Ubuntu 固件镜像（~4 GB） |
| `rk3576_data/5-DevelopmentTool/` | Windows 烧录工具全家桶 |
| `rk3576_data/3-SoftwareData/GCC_Cross_toolchains/` | ARM64 交叉编译工具链 |
| `rk3576_data/3-SoftwareData/RKNPU/rknn-toolkit2.zip` | NPU 模型转换工具（~700 MB） |
| `rk3576_data/3-SoftwareData/RKNPU/rknpu2.zip` | NPU 板端运行时（~500 MB） |
| `rk3576_data/3-SoftwareData/Linux_rknn_yolov5/rknn_model_zoo.tar.gz` | RKNN 模型库（~400 MB） |

> 下载地址：[KICKPI 文档中心](https://doc.kickpi.cn) → 产品 → K7 → 下载

---

## 资源链接

- KICKPI 官网（中文）: https://kickpi.cn
- KICKPI 文档/下载: https://doc.kickpi.cn
- KICKPI 官网（英文）: https://www.kickpi.com
