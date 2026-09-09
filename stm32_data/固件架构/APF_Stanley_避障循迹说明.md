# APF + Stanley 避障循迹 —— 手把手说明

> 本文讲解 K7 小车 STM32 固件里「APF 避障 + Stanley 循迹」这套功能的实现。
> 代码在固件工程 `STM32F407VET6_src/`（仓库根目录）下，主要在 `BALANCE/` 目录。
> 目标读者：第一次接触这套代码、想知道「它怎么跑起来的、每个函数干嘛、参数在哪调」的同学。

---

## 一、这套功能是干什么的

小车在 STM32 上**自己闭环**跑两件事：

1. **循迹（Stanley）**：让车沿一条**参考路径**走。当前参考是**沿 +x 的直线段**：从起点 (0,0) 直行 **3 米**（`STANLEY_LINE_LEN`），走满里程后自动停车。车偏离直线了、车头方向不对了，Stanley 算出该转多少把它拉回直线上。
2. **避障（APF）**：车头的三路测距传感器（前 / 左前 45° / 右前 45°）发现障碍物时，产生一个「排斥力」，让车往没障碍的方向绕开，同时减速。（早期版本参考路径是逆时针圆 R=0.6m，2026-09-06 起改为直线 3m。）

**关键点**：这套东西**全在 STM32 上跑**，不经过 RK3576，所以没有串口往返延迟。控制频率 50Hz（每秒算 50 次），够快够稳。

### 一句话数据流

```
三路测距(rangerA/B/C) ──┐
编码器轮速 → 里程计(x,y,θ) ─┼──→ Stanley循迹 + APF避障 → 速度v + 角速度ω
                          └──→ robot_control.Vx/Vz → balance_task 驱动电机
```

---

## 二、文件地图（去哪看什么）

| 想了解什么 | 去哪个文件 | 位置 |
|-----------|-----------|------|
| 循迹算法（Stanley） | `stanley.c` | `BALANCE/` |
| 循迹参数（增益、直线长） | `stanley.h` | `BALANCE/Inc/` |
| 避障算法（APF 斥力） | `apf.c` | `BALANCE/` |
| 避障参数（斥力、量程、速度） | `apf.h` | `BALANCE/Inc/` |
| 任务编排（读传感器→算→写电机） | `apf_task.c` | `BALANCE/` |
| 任务频率/优先级 | `apf_task.h` | `BALANCE/Inc/` |
| 任务注册（启动时创建任务） | `main.c` | `USER/` |
| 三路测距数据从哪来 | `gy53_pwm.c` | `HARDWARE/` |
| 电机怎么被驱动 | `balance_task.c` | `BALANCE/` |

> 说明：`apf.h` 里定义了共用的数据类型（`APF_Car`）和所有参数宏；`apf.c`、`stanley.c` 是实现（`.c` 文件）；`apf_task.c` 是「总调度」，把它们串起来。

---

## 三、逐文件讲解

### 3.1 `BALANCE/Inc/apf.h` —— 类型和参数（先看这个）

这个头文件定义了两样东西：**数据类型** 和 **可调参数**。

```c
/* 小车位姿（里程计系） */
typedef struct {
    float x;      // 里程计 X (m)，前进方向
    float y;      // 里程计 Y (m)，左方向
    float theta;  // 偏航角 (rad)，0=朝 +x，逆时针为正
    float v;      // 当前线速度 (m/s)
} APF_Car;
```

**坐标约定**（很重要，看懂它才看得懂算法）：
- `x` = 前进方向，`y` = 左方向，`theta` = 车头朝向，逆时针为正。
- 车一开始在 `(0, 0)`，车头朝 `+x` 方向。

**可调参数**（都在这里改）：

```c
#define APF_K_REP      2.0f    /* 斥力增益：越大绕障越猛 */
#define APF_RHO_0      0.9f    /* 斥力作用半径 (m)：障碍在此范围内才产生斥力 */
#define APF_SAFETY_DIST 0.15f  /* 安全距离 (m) */
#define APF_W_MAX      1.5f    /* 最大角速度限幅 (rad/s) */
#define APF_V_MAX      0.3f    /* 最大线速度 (m/s) */
#define APF_V_MIN      0.13f   /* 最小线速度 */
#define APF_WHEEL_BASE 0.329f  /* 轮距 (m)，差分运动学用 */

/* "正前近障朝空侧绕"：前方障碍正中、两侧还没探到时也能提前转向（1=开 0=关）*/
#define APF_FRONT_STEER_EN      1
#define APF_FRONT_STEER_MAX     0.6f   /* 该项最大转向角速度 (rad/s) */
#define APF_FRONT_SIDE_DEAD     0.12f  /* 左右读数差(m)<此值视为"居中" */
#define APF_FRONT_DEFAULT_DIR   1      /* 居中兜底绕向：1=左绕, -1=右绕 */

/* 三路传感器角度（车身系，弧度）*/
#define SENSOR_ANGLE_FRONT   0.0f              /* 前 0° */
#define SENSOR_ANGLE_LEFT    0.7853981634f     /* 左前 +45° */
#define SENSOR_ANGLE_RIGHT  -0.7853981634f     /* 右前 -45° */
```

> **调参第一站**：想改「绕障猛不猛」调 `APF_K_REP`；想改「多远开始避」调 `APF_RHO_0`（实测 0.9 比默认 0.5 触发早很多）；障碍物若总是**正中顶着才绕**，说明 ±45° 探不到，开 `APF_FRONT_STEER_EN=1` 用前方直接转向；绕错方向就翻 `APF_FRONT_DEFAULT_DIR` 或 `-sin` 符号。想改「跑多快」调 `APF_V_MAX/V_MIN`。

### 3.2 `BALANCE/Inc/stanley.h` —— 循迹参数

```c
#define STANLEY_K_PSI       0.6f   /* 航向误差增益 */
#define STANLEY_K_CTE       0.8f   /* 横向误差增益 */

#define STANLEY_LINE_LEN      3.0f   /* 直线行驶长度 (m)，里程 ≥ 此值即停车 */
#define STANLEY_LINE_HEADING  0.0f   /* 直线方向 (rad)：沿 +x */
```

> **调参第二站**：直线走多长改 `STANLEY_LINE_LEN`（想一直走就改大）；循迹「跟得紧不紧、会不会抖」调 `K_PSI/K_CTE`（实测调小到 0.6/0.8 后过障回线过冲明显减小）。以后要把「硬编码直线」换成「RK3576 下发路径」，就改 `stanley.c` 里算参考点的那几行。

### 3.3 `BALANCE/stanley.c` —— 循迹算法实现（核心）

只有 **一个函数** `stanley_steering()`，每个周期调用一次，输入车位姿，输出该转的角速度。

```c
float stanley_steering(const APF_Car *car)
{
    /* 1. 航向误差：参考方向 = +x(0 rad)，psi_e = 0 - theta */
    float psi_e = wrap_pi(STANLEY_LINE_HEADING - car->theta);

    /* 2. 横向误差：到直线 y=0 的有向距离（偏左 +y 取负，使左偏需右回） */
    float cte = -car->y;

    /* 3. Stanley 公式：航向误差 + 横向误差修正 */
    float v = (car->v > 0.15f) ? car->v : 0.15f;   // 防除零
    float omega = STANLEY_K_PSI * psi_e
                + STANLEY_K_CTE * atan2f(cte, v);
    return omega;
}
```

**逐行解释**：
- **第 1 步**：参考方向是 `+x`（角度 0）。`psi_e` = 「该走的方向(0) − 车头方向」，车头左偏（θ>0）时它为负 → 指令右回。
- **第 2 步**：`cte`（cross-track error，横向误差）是「车到直线 y=0 的距离」。车偏左（+y）时取负 → 指令把车往右拉回直线。
- **第 3 步**：**Stanley 核心公式** `ω = K_psi·ψe + K_cte·atan(cte/v)`。前半段纠正车头方向，后半段纠正横向位置。`atan` 让横向修正有上限（不会无限增大），所以平滑。

> **看懂这一句就懂了循迹**：Stanley = 「车头摆正」+「往直线上拉」，两股力加权求和。
> **停车的另一半逻辑在 `apf_task.c`**：每周期把编码器实际速度 `v` 积分进 `dist`，`dist ≥ STANLEY_LINE_LEN` 时把 `v_cmd/ω_cmd` 清零并一直保持——所以车走满 3m 会自动停。
> 若实车转向方向反了（车往错误一侧越偏越远），把 `stanley.h` 里 `STANLEY_K_PSI` 和 `STANLEY_K_CTE` 同时取负。

### 3.4 `BALANCE/apf.c` —— 避障算法实现（核心）

两个函数：`rep_force()`（算单个传感器的斥力大小）和 `apf_repulsive()`（三路合成转向）。

```c
/* 单个传感器：距离 d 越近，斥力越大；d >= RHO_0 时斥力=0 */
static float rep_force(float d)
{
    if (d >= APF_RHO_0) return 0.0f;         // 超出作用范围，不产生斥力
    if (d < 0.02f) d = 0.02f;               // 防除零
    float inv = 1.0f/d - 1.0f/APF_RHO_0;    // (1/d - 1/ρ0)
    return APF_K_REP * inv / (d*d);         // K * (1/d - 1/ρ0) / d²
}

float apf_repulsive(float dist_front, float dist_left, float dist_right)
{
    float omega = 0.0f;

    /* 每路传感器在「障碍物方向」产生斥力，横向分量(-sinφ)决定转向 */
    omega += rep_force(dist_front) * (-sinf(SENSOR_ANGLE_FRONT));   // 前方：只减速不转向
    omega += rep_force(dist_left)  * (-sinf(SENSOR_ANGLE_LEFT));    // 左前障碍 → 右转
    omega += rep_force(dist_right) * (-sinf(SENSOR_ANGLE_RIGHT));   // 右前障碍 → 左转

    return clampf(omega, -APF_W_MAX, APF_W_MAX);   // 限幅
}
```

**逐行解释**：
- `rep_force()` 就是经典人工势场的斥力公式：障碍越近（`d` 越小），`(1/d - 1/ρ0)/d²` 越大，斥力越强。超出作用半径 `ρ0` 就归零（看不见 = 不避）。
- `apf_repulsive()` 把三路斥力的**横向分量**加起来得到转向角速度：
  - 前方障碍（角度 0）→ `-sin(0)=0`，不产生横向转向，只会让车减速（见下一段）。
  - 左前障碍（+45°）→ `-sin(45°)<0`，是**负角速度**（右转），把车往右推、绕开左边的障碍。
  - 右前障碍（-45°）→ `-sin(-45°)>0`，**正角速度**（左转），绕开右边的障碍。

```c
/* 遇障减速：越近越慢 */
float apf_speed_limit(float dist_min)
{
    if (dist_min >= APF_RHO_0) return APF_V_MAX;    // 无障碍，全速
    if (dist_min <= APF_SAFETY_DIST) return APF_V_MIN;  // 太近，最低速
    float t = (dist_min - APF_SAFETY_DIST) / (APF_RHO_0 - APF_SAFETY_DIST);
    return APF_V_MIN + (APF_V_MAX - APF_V_MIN) * t;  // 中间线性插值
}
```

> **看懂这两句就懂了避障**：斥力公式决定「离障碍越近推力越大」；`-sinφ` 决定「左边障碍往右绕、右边障碍往左绕」；`apf_speed_limit` 决定「越近越慢」。

### 3.5 `BALANCE/apf_task.c` —— 总调度（把上面串起来）

这是 FreeRTOS 任务，50Hz 循环执行：

```c
void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    const float dt = 1.0f / (float)APF_TASK_RATE;   // 0.02s = 20ms

    APF_Car car;                    // 小车位姿，初始 (0,0,0)
    car.x = 0; car.y = 0; car.theta = 0; car.v = 0;

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(APF_TASK_RATE));  // 固定 50Hz 节拍

        /* 1. 读三路测距（米，5.0=无遮挡） */
        float d_front = s21c_board.rangerA;   // 前
        float d_left  = s21c_board.rangerB;   // 左前
        float d_right = s21c_board.rangerC;   // 右前

        /* 2. 编码器里程计：轮速 → v/ω → 积分位姿 */
        float v_left  = robot.MOTOR_A.Encoder;   // 左轮速度 m/s
        float v_right = robot.MOTOR_B.Encoder;   // 右轮速度 m/s
        float v       = (v_left + v_right) * 0.5f;             // 线速度 = 两轮平均
        float omega   = (v_right - v_left) / APF_WHEEL_BASE;   // 角速度 = 轮速差/轮距
        car.theta += omega * dt;            // 积分车头方向
        car.x     += v * cosf(car.theta) * dt;  // 积分 X
        car.y     += v * sinf(car.theta) * dt;  // 积分 Y
        car.v      = v;

        /* 3. Stanley 循迹 + APF 避障（转向叠加） */
        float omega_cmd = stanley_steering(&car) + apf_repulsive(d_front, d_left, d_right);

        /* 4. 速度：遇障减速 */
        float v_cmd = apf_speed_limit(min3(d_front, d_left, d_right));

        /* 5. 写控制 */
        robot_control.Vx = clampf(v_cmd, -APF_V_MAX, APF_V_MAX);     // 线速度
        robot_control.Vz = clampf(omega_cmd, -APF_W_MAX, APF_W_MAX); // 角速度
        robot_control.ControlMode = 0;   // 0=直驱模式，balance_task 直接执行 Vx/Vz
    }
}
```

**逐段解释**：
- **第 1 段**：`s21c_board.rangerA/B/C` 是 `gy53_pwm.c` 里测出来的三路距离（单位米，5.0 表示没障碍）。A=前、B=左前、C=右前。
- **第 2 段**：这是**里程计**——用编码器测的左右轮速，算出整车速度 `v` 和角速度 `ω`，再积分出位姿 `(x, y, theta)`。Stanley 循迹要靠这个 `(x,y,θ)` 才知道车在哪、朝哪。
- **第 3 段**：**核心融合**——把 Stanley 算的循迹转向 + APF 算的避障转向**相加**，就是最终角速度。
- **第 4 段**：速度取三路里最近距离来决定快慢。
- **第 5 段**：把结果写进 `robot_control.Vx/Vz`，`balance_task`（100Hz 的电机控制任务）会拿去驱动电机。`ControlMode=0` 表示「直驱模式」，让 balance_task 直接用我们写的 Vx/Vz。

### 3.6 `USER/main.c` —— 注册任务

在 `start_task()` 里多了一行，把 APF 任务创建出来：

```c
CreateTaskChecked(APF_task, "APF_task", APF_STK_SIZE, NULL, APF_TASK_PRIO, NULL, SYSTEM_DIAG_TASK_APF);
```

> 任务创建后，FreeRTOS 调度器会以 `APF_TASK_PRIO=5` 的优先级、`APF_TASK_RATE=50Hz` 的频率周期调用 `APF_task`。

---

## 四、参数速查表（改哪里）

| 想达到的效果 | 改哪个参数 | 在哪个文件 | 默认值 |
|-------------|-----------|-----------|--------|
| 直线走多长 / 想改终点 | `STANLEY_LINE_LEN` | `stanley.h` | 3.0 |
| 直线方向 | `STANLEY_LINE_HEADING` | `stanley.h` | 0（+x） |
| 循迹更紧/更抖 | `STANLEY_K_PSI`、`STANLEY_K_CTE` | `stanley.h` | 0.6 / 0.8 |
| 绕障更猛/更柔 | `APF_K_REP` | `apf.h` | 2.0 |
| 多远开始避障 | `APF_RHO_0` | `apf.h` | 0.9 |
| 安全距离 | `APF_SAFETY_DIST` | `apf.h` | 0.15 |
| 正前近障自动转向 | `APF_FRONT_STEER_EN` | `apf.h` | 1（开） |
| 该项最大转向 | `APF_FRONT_STEER_MAX` | `apf.h` | 0.6 |
| 居中兜底绕向 | `APF_FRONT_DEFAULT_DIR` | `apf.h` | 1（左绕） |
| 最快/最慢速度 | `APF_V_MAX` / `APF_V_MIN` | `apf.h` | 0.3 / 0.13 |
| 最大角速度 | `APF_W_MAX` | `apf.h` | 1.5 |
| 轮距（里程计） | `APF_WHEEL_BASE` | `apf.h` | 0.329 |
| 控制频率 | `APF_TASK_RATE` | `apf_task.h` | 50Hz |

---

## 五、常见问题排查

### 1. 车一启动就往反方向转 / 往外跑
这是**转向正负号反了**。把 `stanley.h` 里的 `STANLEY_K_PSI` 和 `STANLEY_K_CTE` 都取负号：
```c
#define STANLEY_K_PSI   -0.6f
#define STANLEY_K_CTE   -0.8f
```

### 2. 避障绕错方向（左前有障碍却往左拐）
把 `apf.c` 里的 `-sinf(...)` 改成 `+sinf(...)`（三处都改）。若用的是新增的"正前近障转向"，把 `APF_FRONT_DEFAULT_DIR` 取反（1↔−1）。

### 3. 车根本不走
检查：
- `robot.MOTOR_A.Encoder` / `MOTOR_B.Encoder` 是左/右轮有没有搞反（搞反了里程计积分会乱，但车应该还能动）。
- `robot_control.ControlMode = 0` 是不是被别的代码改掉了（比如 RK3576 还在下发 cmd_vel 会设成 `_APP_Control`，抢控制权）。**测 APF 时别跑 RK3576 的 cmd_vel/twist_mux**。
- 三路测距是不是一直读 0（gy53_pwm.c 没正常工作），导致 `apf_speed_limit` 一直压到最低速。

### 4. 直行时越走越偏（不沿直线）
- 里程计漂移：编码器积分会慢慢累积误差，这是正常现象。
- 可先调大 `STANLEY_K_CTE` 让横向修正更强，把车「拉回直线上」；若车是**越偏越往错的方向拐**，那是转向符号反了，把 `STANLEY_K_PSI/K_CTE` 同时取负。

---

## 六、以后怎么扩展

1. **换参考路径**（把直线改长度、或接 RK3576 下发路点）：改 `stanley.c` 里算参考点/`cte`/航向误差的几行（直线现在只需距离），需要多段或曲线时改成「查最近路点 + 路点切线」。
2. **加双目相机避障**：在 `apf_task.c` 里再多加一路「相机障碍」，和现有的三路测距斥力叠加，**不用删三路测距**——它们是 STM32 上的快速安全层，相机是 RK3576 上的全局层。
3. **调参自动化**：把参数放到串口/蓝牙可下发，避免每次改宏重烧固件。

---

## 七、实车调参：记录数据 + 看什么（0xFB 调试帧，2026-09-06）

STM32 上电自跑时，控制器**内部**的量（指令速度、Stanley/APF 分量）原本不外发，没法看。为此加了一条**自定义上行调试帧 0xFB**，把小车在跑什么、控制器怎么想的，实时发到 K7 上记录。

### 7.1 帧里有什么（对应文件：`apf_task.c` 采样、`data_task.c` 组装）

| 数据 | 含义 | 调什么参数用它 |
|------|------|----------------|
| `x, y, θ` | 里程计位姿 | 画实际轨迹 vs 参考圆 |
| `cte, ψe`（K7 侧算出） | 横向误差 / 航向误差 | `STANLEY_K_PSI`、`STANLEY_K_CTE` |
| `v_cmd` | 指令线速度 | `APF_V_MAX/MIN` 限幅是否生效 |
| `w_cmd / w_stanley / w_apf` | 总角速度 = 循迹 + 避障分量 | 看转弯**是不是 APF 推过头**、避障猛不猛 → `APF_K_REP/RHO_0` |
| `v_act / w_act` | 实际速度 | 指令和实际是否跟上（饱和/打滑） |
| `dA/dB/dC` | 三路测距 | 遇障时传感器实际读数、触发点 |

数据流：`apf_task` 每 50Hz 采样进 `g_apf_debug` → `data_task` 每 20Hz 组装 **0xFB 帧**（27B，int16 大端 ×1000，BCC）经 USART3 上行 → K7 `k7_serial_node` 解析发布 **`/apf_debug`**（`k7_msgs/ApfDebug`）。

> 只在这份**带 APF_task 的固件**里才发：`g_apf_debug_valid` 置 1 才发。从机 ROS 模式固件没有 APF_task，不会发、不干扰。

### 7.2 怎么记录（ROS 侧，K7 板）

固件烧好自跑后，在 K7 上（工作区 `colcon build --symlink-install` 过）一条命令：

```bash
./run/record_apf.sh            # = listen-only 串口节点 + apf_recorder，CSV 落在 ~/apf_log/
```

脚本内部关键点：`k7_serial_node` 以 **`enable_downlink:=false`**（只收不发）运行——避免它那 10Hz 零速看门狗帧把 STM32 从 APF 自跑切到串口模式。对应代码在 `k7_bringup`（`enable_downlink` 参数 + `k7_serial_node.cpp` 的 `Send_Cmd_Vel` 开头判断、`k7_robot.h` 成员）。

CSV 列：`t, x, y, theta, cte, psi_e, v_cmd, w_cmd, w_stanley, w_apf, v_act, w_act, d_front, d_left, d_right`。

### 7.3 怎么画（PC 或 K7 上都行）

```bash
# 在装有 k7_apf_debug 的机器（PC 上跑则先 colcon build，或直接在 K7 上）：
ros2 run k7_apf_debug plot_apf ~/apf_log/apf_xxxx.csv        # 存 png
ros2 run k7_apf_debug plot_apf ~/apf_log/apf_xxxx.csv --interactive  # PC 上有显示时弹窗
```

### 7.4 看图怎么调（速查）

| 现象 | 看哪张图 | 怎么调 |
|------|---------|--------|
| 实际轨迹比圆小/大一圈（恒定 cte 不为 0） | 左上 轨迹图 / 右上 cte | 先检查起点和圆心；仍偏大改 `K_CTE` |
| cte 正弦来回、车"画蛇" | 右上 cte 曲线 | `K_PSI`/`K_CTE` 太大，调小 |
| 压弯、切线跟不上（cte 波动大） | 右上 cte/ψe | `K_CTE`、`K_PSI` 调大 |
| 遇障时 w_apf 一下顶到限幅、太猛 | 右下 w 分量 | `APF_K_REP` 调小、或 `APF_RHO_0` 调小 |
| 绕障不够、贴太近（d_xxx 已很小 w_apf 还小） | 右下 w 分量 + 距离图 | `APF_K_REP` 调大 |
| v_cmd 长期顶在 V_MAX 下不来 | 左下 v 图 | 场地太小，改小 `APF_V_MAX` |
| v_act/w_act 跟不上指令 | 左下/右下 | 检查是否打滑或 `APF_V_MIN` 太小 |

> 参考路径类型由 `apf_recorder` 的 `reference` 参数控制（默认 `circle`）。**当前固件参考是直线**，记录时应传 `reference:=line`（`run/record_apf.sh` 第二参数传 `line`），cte/ψe 才按直线算；画图用 `plot_apf xxx.csv --ref line --line-len 3`。若固件改回参考圆，记录用默认 `circle` 且 `circle_radius` 与 `stanley.h` 一致（原默认 0.6m）。

### 7.5 相关文件速查

| 文件 | 位置 | 作用 |
|------|------|------|
| 0xFB 帧数据源结构 `APF_Debug_t` | 固件 `BALANCE/Inc/apf.h` | 定义采样结构 + extern |
| 每 50Hz 采样填入 | 固件 `BALANCE/apf_task.c` | 步骤 6 |
| 20Hz 组装 + USART3 发送 | 固件 `BALANCE/data_task.c` | `apfdbgbuffer` / `Usart3_SendTask` |
| 消息定义 | ROS `k7_msgs/msg/ApfDebug.msg` | `/apf_debug` 话题类型 |
| 串口解析 + 发布 `/apf_debug` | ROS `k7_bringup` `k7_serial_node` | 状态机 `frame_type==4`，`Publish_ApfDebug`；`enable_downlink` 纯记录开关 |
| CSV 记录 + 算 cte/ψe | ROS `k7_apf_debug` `apf_recorder` | `ros2 run k7_apf_debug apf_recorder` |
| 一键记录 launch/脚本 | ROS `k7_apf_debug/launch/record.launch.py`、`run/record_apf.sh` | |
| 绘图 | ROS `k7_apf_debug` `plot_apf_log.py` | `ros2 run k7_apf_debug plot_apf xxx.csv` |

---

> 一句话总结：**`apf_task.c` 是大脑（调度），`stanley.c` 管循迹，`apf.c` 管避障，`apf.h`/`stanley.h` 是旋钮（参数），`main.c` 是开关（注册任务）；要"看清旋钮转得对不对"，STM32 发 0xFB 调试帧 → K7 记 CSV → `plot_apf` 看图调参。**
