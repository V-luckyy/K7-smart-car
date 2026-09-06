# APF + Stanley 避障循迹 —— 手把手说明

> 本文讲解 K7 小车 STM32 固件里「APF 避障 + Stanley 循迹」这套功能的实现。
> 代码在固件工程 `WHEELTEC_C50X_2026.05.29_GY53_PWM/` 下，主要在 `BALANCE/` 目录。
> 目标读者：第一次接触这套代码、想知道「它怎么跑起来的、每个函数干嘛、参数在哪调」的同学。

---

## 一、这套功能是干什么的

小车在 STM32 上**自己闭环**跑两件事：

1. **循迹（Stanley）**：让车沿着一个**圆**走（圆半径 0.6m，圆心在里程计原点）。车偏离圆了、车头方向不对了，Stanley 算出该转多少。
2. **避障（APF）**：车头的三路测距传感器（前 / 左前 45° / 右前 45°）发现障碍物时，产生一个「排斥力」，让车往没障碍的方向绕开，同时减速。

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
| 循迹参数（增益、圆半径） | `stanley.h` | `BALANCE/Inc/` |
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
#define APF_RHO_0      0.5f    /* 斥力作用半径 (m)：障碍在此范围内才产生斥力 */
#define APF_SAFETY_DIST 0.15f  /* 安全距离 (m) */
#define APF_W_MAX      1.5f    /* 最大角速度限幅 (rad/s) */
#define APF_V_MAX      0.3f    /* 最大线速度 (m/s) */
#define APF_V_MIN      0.13f   /* 最小线速度 */
#define APF_WHEEL_BASE 0.329f  /* 轮距 (m)，差分运动学用 */

/* 三路传感器角度（车身系，弧度）*/
#define SENSOR_ANGLE_FRONT   0.0f              /* 前 0° */
#define SENSOR_ANGLE_LEFT    0.7853981634f     /* 左前 +45° */
#define SENSOR_ANGLE_RIGHT  -0.7853981634f     /* 右前 -45° */
```

> **调参第一站**：想改「绕障猛不猛」调 `APF_K_REP`；想改「多远开始避」调 `APF_RHO_0`；想改「跑多快」调 `APF_V_MAX/V_MIN`。

### 3.2 `BALANCE/Inc/stanley.h` —— 循迹参数

```c
#define STANLEY_K_PSI     1.0f    /* 航向误差增益 */
#define STANLEY_K_CTE     1.5f    /* 横向误差增益 */

#define STANLEY_CIRCLE_R   0.6f   /* 圆半径 (m) */
#define STANLEY_CIRCLE_CX  0.0f   /* 圆心 X */
#define STANLEY_CIRCLE_CY  0.0f   /* 圆心 Y */
```

> **调参第二站**：圆多大改 `STANLEY_CIRCLE_R`；循迹「跟得紧不紧、会不会抖」调 `K_PSI/K_CTE`。以后要把「硬编码圆」换成「RK3576 下发路径」，就改 `stanley.c` 里算参考点的那几行。

### 3.3 `BALANCE/stanley.c` —— 循迹算法实现（核心）

只有 **一个函数** `stanley_steering()`，每个周期调用一次，输入车位姿，输出该转的角速度。

```c
float stanley_steering(const APF_Car *car)
{
    /* 1. 车相对圆心的位置 */
    float dx = car->x - STANLEY_CIRCLE_CX;   // 车心到圆心 X 差
    float dy = car->y - STANLEY_CIRCLE_CY;   // 车心到圆心 Y 差
    float r  = sqrtf(dx*dx + dy*dy);         // 车到圆心的距离

    /* 2. 横向误差：正=车在圆外，负=车在圆内 */
    float cte = r - STANLEY_CIRCLE_R;

    /* 3. 圆上最近点的切线方向（逆时针圆的切线） */
    float tangent = atan2f(-dy, dx);

    /* 4. 航向误差 = 期望切线方向 - 车头方向，归一化到 [-π,π] */
    float psi_e = wrap_pi(tangent - car->theta);

    /* 5. Stanley 公式：航向误差 + 横向误差修正 */
    float v = (car->v > 0.15f) ? car->v : 0.15f;   // 防除零
    float omega = STANLEY_K_PSI * psi_e
                + STANLEY_K_CTE * atan2f(cte, v);
    return omega;
}
```

**逐行解释**：
- **第 1 步**：算车在圆外还是圆内、离圆多远。
- **第 2 步**：`cte`（cross-track error，横向误差）是「车离圆周的距离」，车在圆外是正、圆内是负。
- **第 3 步**：算车应该朝哪个方向走（圆的切线方向）。
- **第 4 步**：`psi_e` 是「车头方向」和「该走方向」的夹角，这个角越大车转得越猛。
- **第 5 步**：**Stanley 核心公式** `ω = K_psi·ψe + K_cte·atan(cte/v)`。前半段纠正车头方向，后半段纠正横向位置。`atan` 让横向修正有上限（不会无限增大），所以平滑。

> **看懂这一句就懂了循迹**：Stanley = 「车头摆正」+「往轨道上拉」，两股力加权求和。

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
| 圆更大/更小 | `STANLEY_CIRCLE_R` | `stanley.h` | 0.6 |
| 圆心位置 | `STANLEY_CIRCLE_CX/CY` | `stanley.h` | (0,0) |
| 循迹更紧/更抖 | `STANLEY_K_PSI`、`STANLEY_K_CTE` | `stanley.h` | 1.0 / 1.5 |
| 绕障更猛/更柔 | `APF_K_REP` | `apf.h` | 2.0 |
| 多远开始避障 | `APF_RHO_0` | `apf.h` | 0.5 |
| 安全距离 | `APF_SAFETY_DIST` | `apf.h` | 0.15 |
| 最快/最慢速度 | `APF_V_MAX` / `APF_V_MIN` | `apf.h` | 0.3 / 0.13 |
| 最大角速度 | `APF_W_MAX` | `apf.h` | 1.5 |
| 轮距（里程计） | `APF_WHEEL_BASE` | `apf.h` | 0.329 |
| 控制频率 | `APF_TASK_RATE` | `apf_task.h` | 50Hz |

---

## 五、常见问题排查

### 1. 车一启动就往反方向转 / 往外跑
这是**转向正负号反了**。把 `stanley.h` 里的 `STANLEY_K_PSI` 和 `STANLEY_K_CTE` 都取负号：
```c
#define STANLEY_K_PSI   -1.0f
#define STANLEY_K_CTE   -1.5f
```

### 2. 避障绕错方向（左前有障碍却往左拐）
把 `apf.c` 里的 `-sinf(...)` 改成 `+sinf(...)`（三处都改）。

### 3. 车根本不走
检查：
- `robot.MOTOR_A.Encoder` / `MOTOR_B.Encoder` 是左/右轮有没有搞反（搞反了里程计积分会乱，但车应该还能动）。
- `robot_control.ControlMode = 0` 是不是被别的代码改掉了（比如 RK3576 还在下发 cmd_vel 会设成 `_APP_Control`，抢控制权）。**测 APF 时别跑 RK3576 的 cmd_vel/twist_mux**。
- 三路测距是不是一直读 0（gy53_pwm.c 没正常工作），导致 `apf_speed_limit` 一直压到最低速。

### 4. 车绕圆越绕越偏（不收敛成圆）
- 里程计漂移：编码器积分会慢慢累积误差，这是正常现象，跑几圈会偏。
- 可先调大 `STANLEY_K_CTE` 让横向修正更强，把车「拉回圆上」。

---

## 六、以后怎么扩展

1. **换参考路径**（把圆换成别的轨迹、或接 RK3576 下发路点）：改 `stanley.c` 里算 `cte` 和 `tangent` 的几行，改成「查最近路点 + 路点切线」。
2. **加双目相机避障**：在 `apf_task.c` 里再多加一路「相机障碍」，和现有的三路测距斥力叠加，**不用删三路测距**——它们是 STM32 上的快速安全层，相机是 RK3576 上的全局层。
3. **调参自动化**：把参数放到串口/蓝牙可下发，避免每次改宏重烧固件。

---

> 一句话总结：**`apf_task.c` 是大脑（调度），`stanley.c` 管循迹，`apf.c` 管避障，`apf.h`/`stanley.h` 是旋钮（参数），`main.c` 是开关（注册任务）。**
