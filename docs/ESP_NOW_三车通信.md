# ESP-NOW：一主两从完整工程示例

源码基于仓库 8f0ec57 接入。新增 ESP32 固件与 K7 ROS 桥；未改 STM32 控制协议或编队算法。

## 1. 数据流与职责

```text
car1 task_manager 发布 /car1/fleet/state
  -> car1 espnow_bridge 订阅、编码
  -> USB/CH340
  -> 主车 ESP32 接收串口帧，ESP-NOW 广播
       ├─ car2 ESP32 接收 -> USB -> car2 espnow_bridge -> /car2/fleet/leader_state
       └─ car3 ESP32 接收 -> USB -> car3 espnow_bridge -> /car3/fleet/leader_state

每辆从车 task_manager 订阅本车 fleet/leader_state
  -> 本车 controller_node -> command_guard -> 原 serial_bridge -> STM32
```

主车只发布自己的状态。两辆从车均跟随 car1：例如 car2 距主车 0.8 m，car3 距主车 1.6 m。每车仍独立感知和控制。这是共享主车轨迹的纵向队列；横向并排编队需要另外修改编队算法。

从车 ESP32 无线只接收；接收到的消息以及本板状态通过 USB 发给本车 K7。主车 ESP32 也通过 USB 返回本板状态，便于读取 MAC 和检查发送计数。车间链路单向，本车 USB 链路双向。

## 2. 完整源码入口

| 文件 | 职责 |
|---|---|
| [espnow_bridge.ino](../ESP32_src/espnow_bridge/espnow_bridge.ino) | Wi-Fi/ESP-NOW 初始化、串口解析、主车广播、从车接收、队列与板端状态 |
| [board_config.h](../ESP32_src/espnow_bridge/board_config.h) | 每块板的角色、车辆编号、主车 MAC、信道 |
| [wire_protocol.h](../ESP32_src/espnow_bridge/wire_protocol.h) | ESP32 线协议、CRC、串口流解析 |
| [espnow_protocol.py](../ROS2_src/K7_ros2/src/k7_tasks/k7_tasks/espnow_protocol.py) | K7 编解码、串口重同步、去重与顺序检查 |
| [espnow_bridge.py](../ROS2_src/K7_ros2/src/k7_tasks/k7_tasks/espnow_bridge.py) | ROS 订阅/发布、USB 连接与重连、时效校验、诊断 |
| [Windows 测试工具](../ESP32_src/windows_test/README.md) | 双板 USB 自动识别、模拟状态注入、逐帧核对、暂停与恢复、结果日志 |

K7 使用现有 `k7_tasks` 包，仍为 7 个自有 ROS 包。节点入口已登记在 setup.py；统一 launch 增加 `espnow_role`、`vehicle_id`；统一 robot.yaml 增加 `espnow_bridge` 参数。无需新建消息或修改 `FleetState`。

## 3. 线协议

USB 与无线沿用同一帧，ESP32 原样转发车辆状态。固定 96 字节，小端，IEEE754 浮点。串口按字节流解析，支持半包、粘包、启动日志和损坏后恢复；无线按完整包处理。

| 偏移 | 类型/长度 | 内容 |
|---|---|---|
| 0 | 2 bytes | ASCII K7，帧头 |
| 2 | uint8 | 协议版本 1 |
| 3 | uint8 | 类型 1=FleetState，2=本板状态 |
| 4 | uint16 | 发送车辆编号 |
| 6 | uint16 | 总长度 96 |
| 8 | uint64 | K7 桥进程 boot_id |
| 16 | uint32 | 消息序号，溢出回绕 |
| 20 | int32 | 原始 ROS 时间戳秒 |
| 24 | uint32 | 原始 ROS 时间戳纳秒 |
| 28 | 16 bytes | 原始 task_manager session UUID |
| 44 | float64 | 共同 fleet 坐标 x，m |
| 52 | float64 | 共同 fleet 坐标 y，m |
| 60 | float64 | yaw，rad，逆时针正 |
| 68 | float32 | 车体前向速度，m/s |
| 72 | float32 | 偏航角速度，rad/s |
| 76 | uint8 | 状态：0 IDLE / 1 RUNNING / 2 STOPPING / 3 FAULT |
| 77 | uint8 | frame_id 的 ASCII 字节数，1..16 |
| 78 | 16 bytes | frame_id，默认 fleet，尾部填零 |
| 94 | uint16 | 前 94 字节 CRC-16/CCITT-FALSE |

类型 2 共用 8 字节头，随后依次为 uptime_ms(uint32)、STA MAC(6 bytes)、role(uint8)、channel(uint8)、usb_rx/radio_tx_done/radio_tx_error/radio_rx/rejected(各 uint32)、保留 54 字节、CRC。完整定义见 C++ 结构体和 Python StatusPacket。

原始消息时间戳和任务会话透传：桥进程 boot_id 用来识别序号归零；session UUID 供原编队算法识别主车任务进程重启。接收时重新打时间戳会掩盖源端积压，本实现保留源时间。

## 4. 刷写三块 ESP32

Arduino IDE 安装 `esp32 by Espressif Systems` **3.3.0**，板型选择 **ESP32 Dev Module**，打开 `ESP32_src/espnow_bridge/espnow_bridge.ino`。同目录的两个头文件一起保留。本例使用自带 WiFi/ESP-NOW/FreeRTOS，无额外 Arduino 库。

该示例固定依赖版本，避免发送回调签名随 ESP-IDF 版本变化；源码兼容区分 IDF 5.5 的发送回调参数。主车、car2、car3 三份配置均已在该版本工具链实际编译通过。

修改 `board_config.h`，分别编译、上传：

| 板子 | IS_LEADER | VEHICLE_ID | LEADER_ID | RADIO_CHANNEL |
|---|---|---|---|---|
| car1 | true | 1 | 1 | 6 |
| car2 | false | 2 | 1 | 6 |
| car3 | false | 3 | 1 | 6 |

主车无需配置 LEADER_MAC。从车要填主车 **ESP32 的 STA MAC**，例如实际读到 `24:6F:28:01:02:03` 后填写：

```cpp
constexpr uint8_t LEADER_MAC[6] = {0x24, 0x6F, 0x28, 0x01, 0x02, 0x03};
```

这是示例地址，使用实际地址替换。默认全零 MAC 会使从车拒绝所有无线状态包。主车 MAC 可先刷主车并在 K7 启动桥节点，读取下一节的 `fleet/link`；STA MAC 是配置中的目标，开发板 USB 设备序列号是另一回事。

### 4.1 Windows 双板一键测试

先用 Windows 电脑验证 `电脑 → 主车 USB → ESP-NOW → 从车 USB → 电脑`，再接入 K7。源码入口 `ESP32_src/windows_test/`；详细步骤见[测试工具说明](../ESP32_src/windows_test/README.md)。

两块板已烧录且从车已设置实际主车 STA MAC 后，安装 Python 3.9+，用两根 USB 数据线同时连接主车与一个从车，关闭其他串口程序。双击 `run.cmd`，或在 Git Bash 执行 `bash run.sh`。脚本首次缺少 pyserial 时自动安装到本地目录，按固件状态识别车辆，发送模拟状态 60 秒、暂停 3 秒、恢复 10 秒，保存接收比例、间隔、内容核对和原始帧日志。测试结束释放串口，可保留主车、换另一从车按回车继续。

这一步验收无线和 USB 数据链路；K7 真实车辆状态、ROS 编队与底盘行为在后续部署验收。

## 5. K7 安装和 USB 设备

各 K7 安装 pyserial、构建当前工作区：

```bash
sudo apt-get install python3-serial
cd ~/Code/K7_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select k7_tasks k7_bringup
source install/setup.bash
```

先将仓库 `ROS2_src/K7_ros2/src` 对应更新同步到目标机；以上沿用项目板端工作区路径。已有 `k7_msgs`、`k7_control` 等包须已构建；首次部署按项目指南全量构建。

ESP32 的 USB 插 K7 的 USB 主机口。保持 STM32 原来的 USB 连接。通过下面命令确定 ESP32 串口及稳定路径：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/serial/by-path/
```

把实际 ESP32 路径写入每车配置 `espnow_bridge.device`。例如选用实际存在的 `/dev/serial/by-path/...` 完整路径；CH340 板可能缺少唯一序列号，此时固定 USB 插口并按 by-path 选择。默认 `/dev/k7_espnow` 只是约定别名，本例未安装通用 CH340 udev 规则，以免多个 CH340 设备抢占同一别名。

串口设备权限沿用系统 dialout/udev 设置；启动桥接后由它独占 ESP32 串口。刷写时先停止桥接程序。

仅检查主车 ESP32 USB、读取 STA MAC，可单独运行：

```bash
ros2 run k7_tasks espnow_bridge --ros-args -r __ns:=/car1 \
  -p role:=leader -p vehicle_id:=1 -p device:=/dev/ttyUSB1
```

此处 `/dev/ttyUSB1` 为示例，替换为查到的 ESP32 设备。在另一终端查看：

```bash
ros2 topic echo /car1/fleet/link --once
```

其中 `sta_mac` 是主车 ESP32 地址。每秒会有本板状态，包括 `usb_ready`、MAC、信道、计数器；USB 重连自动重试。确认 MAC 后停止此单独节点，再使用统一 launch，避免重复占用串口。

## 6. K7 接口与启动配置

| 车辆 | 输入 | 输出 |
|---|---|---|
| car1 桥 | /car1/fleet/state：FleetState | USB 状态帧；/car1/fleet/link：DiagnosticArray |
| car2 桥 | USB 收到的主车帧 | /car2/fleet/leader_state：FleetState；/car2/fleet/link |
| car3 桥 | USB 收到的主车帧 | /car3/fleet/leader_state：FleetState；/car3/fleet/link |

`fleet/leader_state` 是你的其他 K7 程序读取主车信息的接口，按常规 ROS 订阅即可；完整从车桥中已实现 `create_publisher(FleetState, 'fleet/leader_state', 1)`。每收到一条通过验证的新状态才发布一次。

每辆车复制完整 robot.yaml 为自己的配置，保留其余标定、底盘参数，在相应字段修改：

```yaml
espnow_bridge:
  device: /dev/k7_espnow  # 替换为本车实际稳定串口路径
  baud: 115200
  leader_id: 1
  channel: 6
  fleet_frame: fleet
  max_age: 0.35
```

各车 `task_manager.fleet_frame` 与桥的 `fleet_frame` 一致。初始同向直线队列的几何示例：

| 车辆 | fleet_x | fleet_y | fleet_yaw | 相对主车的 gap |
|---|---|---|---|---|
| car1 | 0.0 | 0.0 | 0.0 | — |
| car2 | -0.8 | 0.0 | 0.0 | 0.8 |
| car3 | -1.6 | 0.0 | 0.0 | 1.6 |

以上为例子，测量实际起始位姿后设置，确认后在三份配置中设 `task_manager.formation_alignment_confirmed: true`。原 task_manager 在它为 false 时停发本车 fleet 状态；仅打开无线桥并不会自动产生主车状态。

三台 K7 使用一致的实际时间，桥与现有跟随算法按源时间判断时效。ESP-NOW 负责传包；系统时间同步使用已有网络时间服务，运行前确认时间偏差。源状态允许超前最多 50 ms，桥默认接受年龄最多 350 ms；原跟随失联门限 500 ms。所有车使用真实系统时间，仿真时钟不在本例部署范围内。

建议每辆车设置不同 ROS_DOMAIN_ID，车间只走 ESP-NOW，避免原 DDS 主车话题与桥重复传递。以下终端命令均在对应车辆执行，后续 echo/action 终端也用同一个本车 domain。

```bash
# car1
export ROS_DOMAIN_ID=71
ros2 launch k7_bringup robot.launch.py namespace:=car1 profile:=trajectory \
  config:=$HOME/k7/car1.yaml publish_fleet:=true espnow_role:=leader vehicle_id:=1
```

```bash
# car2
export ROS_DOMAIN_ID=72
ros2 launch k7_bringup robot.launch.py namespace:=car2 profile:=formation \
  config:=$HOME/k7/car2.yaml espnow_role:=follower vehicle_id:=2
```

```bash
# car3
export ROS_DOMAIN_ID=73
ros2 launch k7_bringup robot.launch.py namespace:=car3 profile:=formation \
  config:=$HOME/k7/car3.yaml espnow_role:=follower vehicle_id:=3
```

可按原项目规则附加 `test_mode:=no_perception` 做无感知低速测试；其行为和现场限制沿用原编队文档。正常模式使用已验收的本地感知。

从车先确认收到数据：

```bash
# car2
ros2 topic echo /car2/fleet/leader_state --once
ros2 topic echo /car2/fleet/link --once
```

然后分别给两个从车发送跟随任务：

```bash
# car2
ros2 action send_goal /car2/task/follow_leader k7_msgs/action/FollowLeader \
  '{leader_topic: /car2/fleet/leader_state, gap: 0.8, max_speed: 0.10}' --feedback
```

```bash
# car3
ros2 action send_goal /car3/task/follow_leader k7_msgs/action/FollowLeader \
  '{leader_topic: /car3/fleet/leader_state, gap: 1.6, max_speed: 0.10}' --feedback
```

最后给主车发送原项目的路径任务。启动命令仅启动节点，跟随 Action 和主车路径任务仍由操作端明确发起。

## 7. 频率、故障和可观察性

- 主车 task_manager 产生不超过 10 Hz 的真实 FleetState；K7 桥和 ESP32 发送调度均限制到最多 10 Hz，调度可能合并最新值，接收频率以实测为准。每帧无线负载 96 字节，设计负载约每秒 960 字节，未计协议头。
- ESP32 接收回调只校验来源、帧长/CRC并写入单元素队列；普通 loop 负责 USB，避免 Wi-Fi 回调被串口阻塞。单元素状态队列保留最新消息。
- ESP32 主车只转发新 USB 帧；从车只转发新无线帧。任何一端都不会周期重播缓存状态来伪造心跳。
- K7 校验源车辆、帧坐标、有限数值、时间戳、序号/进程身份；旧包、重复包、乱序包被丢弃。缺包计数反映序列间隙，既可能来自无线丢失，也可能来自上游最新值合并。
- USB 拔插自动重连；本板状态超时使桥停止接受/发送状态。跟随任务收不到新鲜主车消息时，沿用现有 leader_timeout 停止并撤销权限；故障恢复后的任务按原状态机重新启动。
- 从车本地重建 `FleetState` 时保留主车 session UUID；主车 task_manager 重启沿用原编队会话检查。整车/里程计重启仍需重新对齐。
- `fleet/link.radio_tx_done` 是主车广播本地发送完成数，无法确认哪辆从车收到。此版本严格无从车无线反馈，因此主车无法根据链路得知从车在线、障碍或故障。
- 本例使用未加密广播和主车 MAC 白名单。MAC 白名单用于筛选来源，CRC 用于检测损坏；它们没有密码学身份认证。需要认证时可升级为两路加密单播并另行确定反馈要求；广播与多播不支持 ESP-NOW 加密。
- 两辆从车都看主车轨迹，car3 无法从通信得知 car2 单独停车。当前靠本车感知/guard处理前方障碍；全队协同停车需要增加状态回传或其他同步机制。

## 8. 验证范围

本机用真实 C++ 协议头与 Python 编解码器交叉检查 96 字节布局，使用已知 CRC 检查向量、坏帧、半包、粘包、启动文本、序号回绕和重复包测试；Linux PTY 验证实际串口写入，ROS 边界以消息/时钟替身检查原时间戳、会话透传和过期拒绝。

Arduino CLI 使用 Arduino-ESP32 3.3.0、FQBN `esp32:esp32:esp32` 实际编译三份配置通过：主车程序 906063 字节、静态 RAM 43612 字节；car2/car3 程序各 901035 字节、静态 RAM 43396 字节。编译命令（已按 Arduino 官方流程安装该版本 core）：

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 ESP32_src/espnow_bridge
```

运行：

```bash
python3 -m unittest discover -s tests -v
```

Windows 工具包含内容错误、重复、乱序、缓存重播、低接收比例和间隔超限检查，并在 Linux 两组真实 PTY 串口上跑通自动识别、发送、暂停、恢复和结果保存。当前本机 54 项回归通过；Windows 实机串口与无线仍待验收。目标机仍需 ROS Jazzy 构建、ESP32 刷写、三板真实无线收发、时钟与串口路径核对、断链停止、坐标对齐、落地队距和避障验收。协议测试和编译通过分别验证本机逻辑与工具链接口，无线性能以实测为准。
