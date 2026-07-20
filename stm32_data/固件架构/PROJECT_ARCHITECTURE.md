# WHEELTEC C50X -- STM32F4 Low-Level Firmware Architecture

> **Date**: 2026-07-20 | **MCU**: STM32F407 (Cortex-M4F) | **RTOS**: FreeRTOS | **IDE**: Keil MDK-ARM

---

## 1. Project Summary

This is the STM32 bottom-layer firmware for the WHEELTEC C50X differential-drive robot chassis. The STM32 serves as the **motion execution layer**; an RK3576 Linux SBC runs ROS2, deep learning inference, and high-level path planning.

### Core Responsibilities

- Motor encoder reading + incremental PI velocity closed-loop (100Hz)
- IMU data acquisition: MPU6050 (HW V1.0) or ICM20948 (HW V1.1+)
- Multi-mode control input: PS2 gamepad, Bluetooth APP, CAN, UART, RC remote
- Ultrasonic/ToF distance sensor acquisition (up to 6 channels, S21C CAN board)
- **APF (Artificial Potential Field) local path planning and obstacle avoidance** (50Hz)
- Robot state telemetry to upper computer via USART1, USART3, and CAN (20Hz)

---

## 2. Build Configuration

A single `#define` in the Keil project selects the car type. Only one may be active at a time:

| Define       | Car Type                | Motors     |
|-------------|-------------------------|------------|
| `DIFF_CAR`  | 2-wheel differential    | 2          |
| `AKM_CAR`   | Ackermann (servo steer) | 2 + servo  |
| `MEC_CAR`   | Mecanum 4-wheel         | 4          |
| `_4WD_CAR`  | 4-wheel differential    | 4          |
| `OMNI_CAR`  | 3-wheel omnidirectional | 3          |

Multiple-definition check in `USER/system.h` triggers a `#error` at compile time.

---

## 3. Directory Map

### 3.1 USER/ -- Entry Point and System Config

| File | Role |
|------|------|
| `main.c` | `main()` -- calls `systemInit()`, creates `start_task`, starts scheduler |
| `system.h` | Master header aggregator -- includes all project headers |
| `system.c` | `systemInit()` -- hardware initialization sequence |
| `stm32f4xx_conf.h` | Peripheral library include filter |
| `stm32f4xx_it.c/h` | Interrupt vector entries (empty shells) |
| `system_stm32f4xx.c/h` | CMSIS system clock init |

> **Note**: `main.c` contains an APF demonstration loop in `while(1)` after `vTaskStartScheduler()`, but this **never executes** because the scheduler does not return. The real APF logic runs in `APF_task()`.

### 3.2 BALANCE/ -- Core Business Logic (Active Build)

#### Headers (BALANCE/Inc/)

| File | Contents |
|------|----------|
| `balance_task.h` | `ROBOT_CONTROL_t` struct, control mode bitmask enums, PI controller typedef, all static function declarations |
| `robot_select_init.h` | `ROBOT` struct, `Robot_Parament_InitTypeDef`, car-type parameter macros |
| `imu_task.h` | `IMU_DATA_t` (gyro + accel + deviation) |
| `data_task.h` | `SEND_DATA` struct (24-byte frame), `S21C_SensorData_t`, BCC check function |
| `show_task.h` | `OLED_t` struct, OLED page display declarations |
| `led_task.h` | LED task config |
| `ps2_task.h` | PS2 controller task config |
| `uartx_callback.h` | `RECEIVE_DATA` struct (11-byte control frame), IP frame constants |
| `can_callback.h` | CAN callback declarations |
| `filter.h` | Filter function declarations |
| `apf.h` | **APF core types**: `APF_Vec2`, `APF_Car`, `APF_CircObs`, `APF_Result`; function `apf_follow()` |
| `apf_task.h` | `APF_task()` declaration, APF_TASK_RATE constant |
| `apf_sensor.h` | Sensor fusion: `APF_SensorFusion()`, `APF_SensorToCircObs()`, sensor mounting positions |
| `sensor_uart.h` | Distance sensor globals: `g_sensor_dist_front/left/right` |
| `sensor_fusion.h` | Thin wrapper that includes `apf_sensor.h` |
| `common_types.h` | **Shared APF types**: `CarState`, `WayPoint`, `SensorObs` |

#### Sources (BALANCE/)

| File | Lines | Key Content |
|------|-------|-------------|
| `balance_task.c` | ~2600 | **100Hz control loop**: encoder feedback, inverse kinematics per car type, incremental PI (A/B/C/D + servo), auto-recharge, self-check, parking, control mode routing |
| `robot_select_init.c` | ~100 | Car type selection (potentiometer ADC -> switch-case) and parameter init |
| `imu_task.c` | ~70 | `MPU6050_task()` and `ICM20948_task()` -- poll IMU at 100Hz |
| `data_task.c` | ~200 | 20Hz telemetry: forward kinematics -> 24-byte frame -> USART1/USART3/CAN1 |
| `show_task.c` | ~350 | 10Hz: OLED page display, APP parameter send, battery voltage + filter |
| `led_task.c` | small | 1Hz LED blink |
| `ps2_task.c` | -- | PS2 SPI read |
| `uartx_callback.c` | ~300 | UART RX ISRs (USART1/2/3/4): 11-byte control frame, Bluetooth AT-command filtering, IP frame |
| `can_callback.c` | ~80 | CAN1 RX0 ISR: speed commands (0x181), auto-recharge (0x182), S21C sensors (0x21C) |
| `filter.c` | -- | Digital filter implementations |
| `apf_task.c` | ~300 | **50Hz APF task**: sense -> odom -> filter -> APF plan -> write Vx/Vz -> CAN report -> OLED |
| `sensor_uart.c` | ~130 | Three sensor UARTs: USART2 (front), USART3 (left), UART5 (right). Frame parsing is **placeholder** |
| `reportErr_task.c` | -- | Error/debug reporting |

### 3.3 APF_MODULE/ -- Standalone APF Development Copy

Functionally identical to the APF code in `BALANCE/` but maintained as a separate module. **Not linked in the current build.**

| File | Equivalent in BALANCE/ |
|------|------------------------|
| `Inc/apf.h`, `Inc/apf_sensor.h`, `Inc/apf_task.h`, `Inc/common_types.h`, `Inc/sensor_fusion.h`, `Inc/sensor_uart.h` | `BALANCE/Inc/apf.h` etc. |
| `Src/apf_task.c`, `Src/sensor_uart.c` | `BALANCE/apf_task.c`, `BALANCE/sensor_uart.c` |

### 3.4 CarType/ -- Alternative Robot Init (Not Active)

Uses a different struct (`ROBOT_t` vs `ROBOT`) that includes `Moto_parameter.Output`. Each car type has its own init file (`diff_robot_init.c`, `akm_robot_init.c`, etc.). The active build uses `BALANCE/robot_select_init.c` instead.

### 3.5 HARDWARE/ -- Board Support Package

| Category | Files |
|----------|-------|
| ADC + GPIO | `adc.c`, `exti.c`, `dma.c` |
| Motor control | `motor.c` (PWM), `encoder.c` (quadrature) |
| Communication | `can.c`, `uartx.c` (TX only), `I2C.c` |
| Storage | `stmflash.c` (internal Flash param storage) |
| Display | `oled.c` |
| User input | `key.c`, `enable_key.c`, `ps2_classic.c`, `remote.c` |
| Actuators | `buzzer.c`, `LED.C`, `bsp_RGBLight.c`, `bsp_gamepad.c` |
| Sensors | `ICM20948/` (9-axis), `MPU6050/` (6-axis) |
| Application | `auto_recharge.c`, `driver_d50a.c`, `DataScope_DP.C` |

### 3.6 FWLIB/ -- STM32F4 Standard Peripheral Library

Standard ST driver files (`stm32f4xx_gpio.c`, `stm32f4xx_usart.c`, `stm32f4xx_tim.c`, `stm32f4xx_can.c`, `stm32f4xx_i2c.c`, etc.)

### 3.7 CORE/ -- CMSIS

`core_cm4.h`, `core_cm4_simd.h`, `arm_math.h`, `startup_stm32f40_41xxx.s`

### 3.8 SYSTEM/ -- System Utilities

| File | Role |
|------|------|
| `delay/delay.c` | Microsecond/millisecond delay (SysTick) |
| `sys/sys.c` | Basic system helpers |
| `usart/usart.c` | UART low-level drivers |

### 3.9 FreeRTOS/

Kernel source. Port layer: `portable/RVDS/ARM_CM4F/`, memory: `heap_4.c`.

### 3.10 MiddleWares/ -- STM32 USB Host Library

USB Host stack supporting HID class. Used by `USB_HOST/`.

### 3.11 USB_HOST/ -- USB Gamepad Application Layer

| File | Role |
|------|------|
| `App/usb_host.c` | USB host initialization and management |
| `App/usbh_hid_GamePad.c` | Generic HID gamepad report parser |
| `App/PS2_gamepad_bk.c` | PS2 Bluetooth gamepad |
| `App/WiredPS2_gamepad.c` | PS2 wired via USB |
| `App/xbox360_gamepad.c` | Xbox 360 controller |
| `App/Flydigi_gamepad_bk.c` | Flydigi gamepad |

---

## 4. FreeRTOS Task Architecture

### 4.1 Task Table

| Task | Rate | Priority | Stack | Responsibility |
|------|------|----------|-------|----------------|
| `start_task` | once | 1 | 512 | Spawns all child tasks, then self-deletes |
| `Balance_task` | **100Hz** | 4 | 512 | **Core motion control**: encoder feedback, inverse kinematics, PI speed loops, self-check, control routing |
| `data_task` | 20Hz | 4 | 512 | Forward kinematics, 24-byte frame assembly, send via USART1/USART3/CAN |
| `show_task` | 10Hz | 3 | 512 | OLED display, battery voltage, APP data, buzzer, low-battery protection |
| `led_task` | 1Hz | 1 | 128 | LED blink |
| `IMU_task` | 100Hz | 5 | 512 | Poll MPU6050 or ICM20948 |
| `APF_task` | **50Hz** | -- | -- | APF perception/planning/control/telemetry/display |
| `D50A_Task` | -- | -- | -- | D50A driver management (HW V1.2 only) |
| `PS2_task` | -- | -- | -- | PS2 SPI read (HW V1.0 only) |
| `ReportErrTask` | -- | Normal | 512 | Error reporting and debug |

### 4.2 Data Flow

```
+-----------------------------------------------+
|              Balance_task (100Hz)              |
|                                               |
|  Encoder(A/B/C/D) --> Inverse Kinematics      |
|      |                     |                  |
|      v                     v                  |
|  robot.MOTOR_*.Encoder  Target_A/B/C/D        |
|                           |                   |
|                           v                   |
|                 Incremental PI Controller      |
|                           |                   |
|                           v                   |
|                      PWM Output               |
|                                               |
|  Shared state: robot_control.{Vx,Vy,Vz}       |
+-----------------------+-----------------------+
                        |
        +---------------+---------------+
        |               |               |
        v               v               v
+-------------+  +-------------+  +--------------+
|  data_task  |  |  show_task  |  |   APF_task   |
|   (20Hz)    |  |   (10Hz)    |  |    (50Hz)    |
|             |  |             |  |              |
| Fwd kin --> |  | OLED pages  |  | Sense: 3x ToF|
| 24B frame   |  | Battery %%   |  | Odom: enc+IMU|
|   |   |     |  | APP printf  |  | Filter(0.35) |
|   v   v     |  | Buzzer mgmt |  | APF: follow()|
| USART1/3    |  +-------------+  | Set Vx, Vz   |
| CAN(0x101)  |                    | CAN report   |
+-------------+                    | OLED 5Hz     |
                                   +--------------+
```

---

## 5. Key Data Structures

### 5.1 Motion Control (`balance_task.h`)

```c
typedef struct {
    u8  ControlMode;          // Bitmask of active control source
    u8  FlagStop;             // 1 = motor disabled
    u8  command_lostcount;    // Lost command counter -> auto-stop at 100
    float Vx, Vy, Vz;         // Target velocities (m/s, rad/s)
    float smooth_Vx, smooth_Vy, smooth_Vz;
    float rc_speed;           // RC reference speed (mm/s), default 500
    float limt_max_speed;     // Speed limit (m/s)
    float smooth_MotorStep;   // Motor speed smoothing step
    uint32_t LineDiffParam;   // Line correction param (0-100)
} ROBOT_CONTROL_t;

typedef struct {
    float Bias, LastBias;     // Current and previous error
    int   Output;             // PWM output
    int   kp, ki;             // PI gains
} PI_CONTROLLER;

typedef struct {
    float Target;             // Target speed (m/s)
    float Encoder;            // Feedback speed (m/s)
} Moto_parameter;
```

### 5.2 Robot Hardware (`robot_select_init.h`)

```c
typedef struct {
    float WheelSpacing;       // Wheel track (m)
    float AxleSpacing;        // Wheelbase (m)
    float Wheel_Circ;         // Wheel circumference (m)
    float GearRatio;          // Motor gear ratio
    uint16_t EncoderAccuracy; // Encoder lines per revolution
    uint8_t  type;            // Car sub-type index
} Robot_Parament_InitTypeDef;
```

### 5.3 IMU (`imu_task.h`)

```c
typedef struct { float x, y, z; } IMU_BASE_t;

typedef struct {
    IMU_BASE_t gyro;            // Angular velocity (deg/s)
    IMU_BASE_t accel;           // Acceleration
    IMU_BASE_t Deviation_gyro;  // Gyro zero-bias
    IMU_BASE_t Deviation_accel; // Accel zero-bias
} IMU_DATA_t;
```

### 5.4 APF Shared Types (`common_types.h`)

```c
typedef struct {
    float x, y;         // Odometer position (m)
    float theta;        // Yaw angle (rad)
    float v, omega;     // Velocity (m/s, rad/s)
    uint32_t tick;      // Timestamp (ms)
} CarState;

typedef struct {
    float x, y;         // Target position (m)
    float theta;        // Target heading (rad, optional)
    uint8_t is_valid;   // 1 = valid
} WayPoint;

typedef struct {
    float x, y;         // Obstacle position (m)
    float confidence;   // [0.0, 1.0]
    uint8_t valid;      // 1 = valid detection
} SensorObs;
```

### 5.5 APF Algorithm Types (`apf.h`)

```c
typedef struct { float x, y; }                    APF_Vec2;
typedef struct { float cx, cy; float confidence; } APF_CircObs;
typedef struct { float x, y, theta; }              APF_Car;
typedef struct { int arrived; }                    APF_Result;
```

---

## 6. Communication Protocols

### 6.1 Uplink (STM32 -> Host): 24-Byte Sensor Frame

Sent at **20Hz** via USART1, USART3, and CAN (split across CAN IDs 0x101/0x102/0x103).

| Offset | Bytes | Contents |
|--------|-------|----------|
| 0 | 1 | Header `0x7B` |
| 1 | 1 | Motor enable flag (0=enabled, 1=disabled) |
| 2-3 | 2 | X-axis velocity x1000, int16 big-endian (mm/s) |
| 4-5 | 2 | Y-axis velocity x1000 |
| 6-7 | 2 | Z-axis angular velocity x1000 |
| 8-9 | 2 | Accelerometer X |
| 10-11 | 2 | Accelerometer Y |
| 12-13 | 2 | Accelerometer Z |
| 14-15 | 2 | Gyroscope X |
| 16-17 | 2 | Gyroscope Y |
| 18-19 | 2 | Gyroscope Z |
| 20-21 | 2 | Battery voltage x1000 |
| 22 | 1 | BCC (XOR of bytes 0-21) |
| 23 | 1 | Footer `0x7D` |

### 6.2 Uplink: Ultrasonic Ranger Frame (Optional)

19 bytes, header `0xFA`, footer `0xFC`. Contains 6 distance channels (float x1000 -> int16). Sent only when S21C ultrasonic board is detected.

### 6.3 Downlink (Host -> STM32): 11-Byte Control Frame

| Offset | Contents |
|--------|----------|
| 0 | Header `0x7B` |
| 1 | Mode: 0=direct speed, 1=nav+IR, 2=nav only, 3=dock speed set, 0xFF=IP frame |
| 2 | Reserved |
| 3-4 | X speed x1000 (int16 big-endian) |
| 5-6 | Y speed x1000 |
| 7-8 | Z angular speed x1000 |
| 9 | BCC (XOR of bytes 0-8) |
| 10 | Footer `0x7D` |

### 6.4 CAN Downlink

| CAN ID | Content |
|--------|---------|
| `0x181` | 3-axis speed command: Vx int16, Vy int16, Vz int16 (x1000 -> m/s) |
| `0x182` | Auto-recharge device control |
| `0x21C` | S21C ultrasonic board data (multi-packet transfer) |
| `0x12345678` | Auto-recharge handshake (extended frame) |

### 6.5 APF Uplink CAN Frames

| CAN ID | Type Byte | Data Layout |
|--------|-----------|-------------|
| `0x100` | `0x02` | Odometry: x*10 (int16), y*10 (int16), theta*1000 (int16), v+128 (u8) |
| `0x100` | `0x03` | Obstacle: x*10 (int16), y*10 (int16), confidence*100 (u8) |

---

## 7. APF Module Deep Dive

### 7.1 File Dependency Chain

```
common_types.h          Shared types (CarState, WayPoint, SensorObs)
    |
    v
apf.h                    APF core types, apf_follow() declaration
    |
    v
apf_sensor.h             APF_SensorFusion(), APF_SensorToCircObs(),
                         sensor mount positions, beam angle constants
    |
    v
sensor_uart.h            g_sensor_dist_front/left/right globals
    |
    v
apf_task.h               APF_task() declaration
    |
    v
apf_task.c               50Hz task: sense -> odom -> filter -> plan -> execute -> report -> display
```

### 7.2 Execution Order (Every 20ms)

1. **Sense**: Read `g_sensor_dist_front/left/right` (populated by UART ISRs)
2. **Odom**: Integrate encoder wheel speeds + IMU yaw -> `CarState`
3. **Filter**: Low-pass filter (alpha=0.35) for smooth pose
4. **Fuse**: `APF_SensorFusion()` converts 3 distances -> 1 virtual obstacle
5. **Plan**: `apf_follow(target, &car, obs, num, &v_cmd, &w_cmd)`
6. **Execute**: Clamp + write to `robot_control.Vx`, `robot_control.Vz`
7. **Report**: CAN frames at 0x100 (odometry + obstacle data)
8. **Display**: OLED update (downsampled to ~5Hz)

### 7.3 APF Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `APF_DT` | 0.01 | Control period (s) |
| `APF_K_ATT` | 1.0 | Attractive gain |
| `APF_K_REP` | 2.0 | Repulsive gain |
| `APF_RHO_0` | 0.5 | Repulsive influence radius (m) |
| `APF_SAFETY_DIST` | 0.15 | Safety distance (m) |
| `APF_V_MAX` | 0.3 | Max linear speed (m/s) |
| `APF_W_MAX` | 1.5 | Max angular speed (rad/s) |
| `APF_GOAL_THRESH` | 0.05 | Arrival threshold (m) |
| `WHEEL_BASE_HALF` | 0.08 | Half wheel track (m) |
| `APF_MAX_OBS` | 8 | Max obstacles tracked |

### 7.4 Odometer Integration

```c
v     = (left_wheel_speed + right_wheel_speed) / 2.0f;
omega = imu.gyro.z * DEG_TO_RAD;       // deg/s -> rad/s
theta += omega * APF_DT_S;             // integrate yaw
x     += v * cosf(theta) * APF_DT_S;   // integrate position
y     += v * sinf(theta) * APF_DT_S;
```

### 7.5 Sensor Hardware Allocation for APF

| Sensor Position | UART | RX Pin | TX Pin | Beam Angle |
|-----------------|------|--------|--------|------------|
| Front           | USART2 | PA3    | PA2    | 0 degrees  |
| Left            | USART3 | PC11   | PC10   | +45 degrees|
| Right           | UART5  | PD2    | PC12   | -45 degrees|

> **Warning**: Frame parsing in `sensor_uart.c` ISRs is a **placeholder** (`parse_sensor_frame()` returns `buf[0]`). Must be replaced with the actual sensor protocol (e.g., TFmini 9-byte frame or VL53L0X-UART).

---

## 8. Sensor Hardware Allocation

### 8.1 Core Sensors

| Sensor | Interface | Pins | Task |
|--------|-----------|------|------|
| MPU6050 | I2C | — | `MPU6050_task()` @ 100Hz |
| ICM20948 | I2C | — | `ICM20948_task()` @ 100Hz |
| Battery voltage | ADC1 | — | `show_task` via `Get_battery_volt()` |
| Car type selector | ADC1 (potentiometer) | — | `Robot_Select()` at init |
| 6x ultrasonic | CAN (S21C board) | — | `can_callback` ISR, data in `s21c_board` |

### 8.2 Communication UARTs

| UART | Pins | Direction | Use |
|------|------|-----------|-----|
| USART1 | PA9(TX)/PA10(RX) | Bidirectional | Debug/control + sensor telemetry TX |
| USART2 | PA2(TX)/PA3(RX) | RX only | Front distance sensor (APF) |
| USART3 | PC10(TX)/PC11(RX) | Bidirectional | ROS control RX + sensor telemetry TX |
| UART4 | PC10(TX)/PC11(RX) or other | RX only | Bluetooth APP + AT command filtering |
| UART5 | PC12(TX)/PD2(RX) | RX only | Right distance sensor (APF) |

---

## 9. Control Mode Routing

```c
#define _ROS_Control    (1<<0)  // USART3
#define _PS2_Control    (1<<1)  // PS2 gamepad
#define _APP_Control    (1<<2)  // Bluetooth APP (UART4)
#define _RC_Control     (1<<3)  // RC remote
#define _CAN_Control    (1<<4)  // CAN bus
#define _USART_Control  (1<<5)  // USART1

// Bitmask: only one mode active at a time
#define Set_Control_Mode(mask)  (robot_control.ControlMode |= (mask),                                  robot_control.ControlMode &= (mask))
#define Get_Control_Mode(mask)  (robot_control.ControlMode & (mask))
```

Priority in `Balance_task`: **Auto-recharge > APP > RC > PS2 > CAN/Serial direct velocity**.

---

## 10. Key Dependencies

| Module | Depends On |
|--------|------------|
| `balance_task.c` | `robot_select_init.h`, `imu_task.h`, all HARDWARE drivers |
| `apf_task.c` | `sensor_uart.h`, `apf.h`, `apf_sensor.h`, `common_types.h`, `balance_task.h`, `imu_task.h`, `can.h`, `show_task.h` |
| `data_task.c` | `robot_select_init.h`, `imu_task.h`, `balance_task.h` |
| `uartx_callback.c` | `data_task.h`, `balance_task.h`, `auto_recharge.h` |
| `can_callback.c` | `balance_task.h`, `auto_recharge.h` |
| `show_task.c` | `imu_task.h`, `bsp_gamepad.h` |

---

## 11. Safety Mechanisms

| Mechanism | Detail |
|-----------|--------|
| Command timeout | `command_lostcount` increments at 100Hz; auto-stops at 100 (~1s without command) |
| Startup delay | `CONTROL_DELAY=1000` ticks (~10s) before external control accepted |
| Low battery cutoff | Motors disabled when voltage < 11.1V |
| Self-check at boot | Spin each motor at 0.5 m/s; verify encoder direction/magnitude |
| BCC checksum | XOR checksum on both uplink (22 bytes) and downlink (9 bytes) frames |
| APF speed clamping | Output v_cmd clamped to [-APF_V_MAX, APF_V_MAX]; omega to [-APF_W_MAX, APF_W_MAX] |

---

## 12. Known Issues / TODOs

1. **Hardcoded waypoint**: `waypoint_get_next()` returns a fixed target `(0, 2.0)`. Must be replaced with data from RK3576 for global path following.

2. **Placeholder sensor parsing**: `parse_sensor_frame()` returns `buf[0]` directly. Actual TFmini/VL53L0X protocol parsing not implemented.

3. **Dead code in main.c**: The `while(1)` APF demo loop after `vTaskStartScheduler()` is unreachable. Remove or refactor.

4. **Two robot init systems**: `BALANCE/robot_select_init.c` (uses `ROBOT`) and `CarType/diff_robot_init.c` (uses `ROBOT_t`). Only the BALANCE version is active.

5. **APF code duplication**: Same code in `APF_MODULE/` and `BALANCE/`. Risk of divergence.

6. **USART2 ISR conflict**: `uartx_callback.c` and `sensor_uart.c` both define `USART2_IRQHandler()`. Only one can be linked. The APF sensor version in `sensor_uart.c` handles D50A data forwarding, but `uartx_callback.c` also uses USART2 for D50A.

---

## 13. Integration with RK3576 Host

### Current Channels

| Channel | Direction | Purpose |
|---------|-----------|---------|
| USART3 | RK3576 -> STM32 | 11-byte speed control frame (ROS mode) |
| USART3 | STM32 -> RK3576 | 24-byte sensor frame + ultrasonic frame |
| CAN 0x181 | RK3576 -> STM32 | 3-axis speed command |
| CAN 0x100 | STM32 -> RK3576 | APF odometry + obstacle info |
| CAN 0x101-0x103 | STM32 -> RK3576 | Sensor frame fragments |

### Expansion Options

To add depth-map obstacles or YOLO detection results from RK3576:

- **Option A**: Extend the 11-byte frame protocol mode byte (e.g., 0x04 = depth obstacle array, 0x05 = detection box)
- **Option B**: Dedicated CAN IDs for structured obstacle/detection packets
- **Option C**: Dedicate a new UART (USART6/UART7) for high-bandwidth data

---

> **Last updated**: 2026-07-20
