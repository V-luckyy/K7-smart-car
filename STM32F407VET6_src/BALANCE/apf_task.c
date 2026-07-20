/**
 * apf_task.c — APF 感知/规划/通信 FreeRTOS 任务
 *
 * 执行顺序:
 *   1. 感知: 读测距传感器 → 取小车状态 → 滤波
 *   2. 控制: 目标点 → 传感器融合 → APF规划 → 运动指令
 *   3. 通信: 通过 CAN 上报小车状态给 ROS
 *   4. 显示: 刷新 OLED APF 信息
 *
 * 底层运动控制由 Balance_task (100Hz) 独立执行,
 * 本任务通过设置 robot_control.Vx / Vz 驱动小车.
 */

#include "apf_task.h"
#include "sensor_uart.h"       /* g_sensor_dist_front/left/right */
#include "sensor_fusion.h"     /* → apf_sensor.h: APF_SensorFusion, APF_SensorToCircObs */
#include "apf.h"               /* apf_follow, APF_Car, APF_CircObs, APF_Vec2 */
#include "common_types.h"      /* CarState, SensorObs, WayPoint */
#include "balance_task.h"      /* robot_control */
#include "imu_task.h"          /* imu */
#include "robot_select_init.h" /* robot */
#include "show_task.h"         /* oled, OLED functions */
#include "can.h"               /* CAN1_Send_Num */

#include <math.h>

/* ====================== 里程计常量 ====================== */
#define APF_DT_S             (1.0f / RATE_50_HZ)   /* 0.02s per cycle */
#define DEG_TO_RAD           0.0174532925f          /* PI / 180 */

/* ====================== 静态状态变量 ====================== */
static CarState g_car = {0};       /* 里程计累积状态 */
static uint8_t  g_inited = 0;

/* ====================== 内部函数 ====================== */

/* ---- 工具函数 ---- */
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int16_t clamp_to_s16(float value, float min_value, float max_value)
{
    value = clampf(value, min_value, max_value);
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void pack_s16(u8 *buf, u8 index, int16_t value)
{
    buf[index]     = (u8)((value >> 8) & 0xff);
    buf[index + 1] = (u8)(value & 0xff);
}

/* ---- 1.2 从编码器 + IMU 拼装小车当前状态 ---- */
/*
 * WHEELTEC 数据源:
 *   robot.MOTOR_A.Encoder / MOTOR_B.Encoder — 轮速 (m/s)
 *   imu.gyro.z — Z轴角速度 (deg/s, 需转换为 rad/s)
 */
static CarState get_car_state(void)
{
    float left_ms  = robot.MOTOR_A.Encoder;   /* 左轮线速度 m/s */
    float right_ms = robot.MOTOR_B.Encoder;   /* 右轮线速度 m/s */
    float v_ms     = (left_ms + right_ms) * 0.5f;  /* 平均线速度 */
    float omega_rads = (float)imu.gyro.z * DEG_TO_RAD; /* deg/s → rad/s */

    g_car.theta += omega_rads * APF_DT_S;    /* 积分偏航角 */
    g_car.x     += v_ms * cosf(g_car.theta) * APF_DT_S;
    g_car.y     += v_ms * sinf(g_car.theta) * APF_DT_S;
    g_car.v      = v_ms;
    g_car.omega  = omega_rads;
    g_car.tick  += (uint32_t)(APF_DT_S * 1000.0f);

    return g_car;
}

/* ---- 1.3 低通滤波, 平滑位姿估计 ---- */
static CarState filter_car_state(CarState raw)
{
    static CarState filt;
    const float alpha = 0.35f;

    if (!g_inited) {
        filt = raw;
        return filt;
    }

    filt.x     += alpha * (raw.x - filt.x);
    filt.y     += alpha * (raw.y - filt.y);
    filt.theta += alpha * (raw.theta - filt.theta);
    filt.v     += alpha * (raw.v - filt.v);
    filt.omega += alpha * (raw.omega - filt.omega);
    filt.tick   = raw.tick;

    return filt;
}

/* ---- 2.1 目标路径点 (硬编码, 后续可接 ROS) ---- */
static WayPoint waypoint_get_next(void)
{
    WayPoint target;
    target.x       = 0.0f;     /* 单位: m */
    target.y       = 2.0f;     /* 2米前方 */
    target.theta   = 0.0f;
    target.is_valid = 1;
    return target;
}

/* ---- 2.4 运动指令 → WHEELTEC 控制接口 ---- */
/*
 * apf.c 输出的 v_cmd 单位为 m/s (APF_V_MAX=0.3),
 * omega_cmd 单位为 rad/s.
 * 直接写入 robot_control.Vx / Vz,
 * Balance_task 的 Drive_Motor() 会在下一个周期执行.
 */
static void apf_cmd_to_robot(float v_cmd, float omega_cmd)
{
    /* 限制最大速度 */
    v_cmd     = clampf(v_cmd, -0.3f, 0.3f);
    omega_cmd = clampf(omega_cmd, -1.5f, 1.5f);

    robot_control.Vx = v_cmd;
    robot_control.Vz = omega_cmd;
}

/* ---- 3.1 CAN 上报小车状态 ---- */
static void ros_send_car_state(CarState state, SensorObs obs)
{
    u8 frame[8];

    /* 帧2 (type=0x02): 里程数据 (x,y,theta,v) */
    frame[0] = 0x02;
    /* x, y 单位 m, 放大10倍后发送 (精度0.1m) */
    pack_s16(frame, 1, clamp_to_s16(state.x * 10.0f, -30000.0f, 30000.0f));
    pack_s16(frame, 3, clamp_to_s16(state.y * 10.0f, -30000.0f, 30000.0f));
    /* theta 单位 rad, 放大1000倍发送 */
    pack_s16(frame, 5, clamp_to_s16(state.theta * 1000.0f, -30000.0f, 30000.0f));
    /* v 单位 m/s, 偏移128后放入1字节 */
    frame[7] = (u8)clampf(state.v + 128.0f, 0.0f, 255.0f);
    CAN1_Send_Num(0x100, frame);

    /* 帧3 (type=0x03): 障碍物 (仅obs.valid时发送) */
    if (obs.valid) {
        frame[0] = 0x03;
        pack_s16(frame, 1, clamp_to_s16(obs.x * 10.0f, -30000.0f, 30000.0f));
        pack_s16(frame, 3, clamp_to_s16(obs.y * 10.0f, -30000.0f, 30000.0f));
        frame[5] = (u8)clampf(obs.confidence * 100.0f, 0.0f, 100.0f);
        frame[6] = 0;
        frame[7] = 0;
        CAN1_Send_Num(0x100, frame);
    }
}

/* ---- 4.1 OLED 显示 APF 信息 ---- */
static void update_oled(CarState state, WayPoint target, SensorObs obs)
{
    static u8 div = 0;

    /* 降频到 ~5Hz 刷新 OLED */
    if (++div < 10) {
        return;
    }
    div = 0;

    OLED_ShowString(0, 0, (const u8 *)"APF");
    OLED_ShowNumber(32, 0, (u32)(target.is_valid), 1, 12);
    OLED_ShowNumber(0, 10, (u32)(fabsf(state.x) * 100.0f), 4, 12);
    OLED_ShowNumber(48, 10, (u32)(fabsf(state.y) * 100.0f), 4, 12);
    /* 显示目标速度 */
    OLED_ShowNumber(0, 20, (u32)fabsf(robot_control.Vx * 100.0f), 4, 12);
    OLED_ShowNumber(48, 20, (u32)fabsf(robot_control.Vz * 100.0f), 4, 12);
    OLED_ShowNumber(0, 30, (u32)(obs.valid), 1, 12);
    OLED_ShowNumber(24, 30, (u32)(fabsf(obs.x) * 100.0f), 3, 12);
    OLED_ShowNumber(64, 30, (u32)(fabsf(obs.y) * 100.0f), 3, 12);
}

/* ====================== APF 主任务 ====================== */

void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while (1)
    {
        /* ---- 固定频率 50Hz ---- */
        vTaskDelayUntil(&lastWakeTime, F2T(APF_TASK_RATE));

        /* ================== 1. 感知组 ================== */

        /* 1.1 读三路测距传感器 */
        float dist_front = g_sensor_dist_front;
        float dist_left  = g_sensor_dist_left;
        float dist_right = g_sensor_dist_right;

        /* 1.2 从编码器 + IMU 拼装小车当前状态 */
        CarState car_raw = get_car_state();

        /* 1.3 低通滤波融合, 输出平滑位姿估计 */
        CarState car_filt = filter_car_state(car_raw);

        /* 标记已初始化 */
        g_inited = 1;

        /* ================== 2. 控制与避障组 ================== */

        /* 2.1 获取目标路径点 */
        WayPoint target = waypoint_get_next();

        /* 2.2 传感器融合 → APF_SensorObs (车身坐标系, m) */
        APF_SensorObs sob = APF_SensorFusion(dist_front, dist_left, dist_right);

        /* 转换SensorObs供通信/显示用 */
        SensorObs obs;
        obs.x          = sob.x;
        obs.y          = sob.y;
        obs.confidence = sob.confidence;
        obs.valid      = (u8)sob.valid;

        /* 2.3 APF 人工势场法 */
        float v_cmd = 0.0f, omega_cmd = 0.0f;
        if (target.is_valid) {
            /* 小车状态 → APF_Car */
            APF_Car apf_car;
            apf_car.x     = car_filt.x;
            apf_car.y     = car_filt.y;
            apf_car.theta = car_filt.theta;

            /* 传感器 → APF_CircObs (车身→世界坐标转换) */
            APF_CircObs apf_obs[1];
            int obs_num = APF_SensorToCircObs(&sob, &apf_car, apf_obs, 1);

            /* 目标点 → APF_Vec2 */
            APF_Vec2 apf_target;
            apf_target.x = target.x;
            apf_target.y = target.y;

            /* 调用 apf.c 的 apf_follow */
            apf_follow(apf_target, &apf_car, apf_obs, obs_num,
                       &v_cmd, &omega_cmd);
        }

        /* 2.4 运动指令 → robot_control */
        apf_cmd_to_robot(v_cmd, omega_cmd);

        /* ================== 3. 通信组 ================== */

        ros_send_car_state(car_filt, obs);

        /* ================== 4. 显示组 ================== */

        update_oled(car_filt, target, obs);
    }
}
