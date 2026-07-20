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
#include "sensor_uart.h"
#include "sensor_fusion.h"
#include "apf.h"
#include "common_types.h"
#include "balance_task.h"
#include "imu_task.h"
#include "robot_select_init.h"
#include "show_task.h"
#include "can.h"

#include <math.h>

/* ====================== 里程计常量 ====================== */
#define APF_DT_S      (1.0f / RATE_50_HZ)    /* 0.02s per cycle */
#define DEG_TO_RAD    0.0174532925f           /* PI / 180 */

/* ====================== 静态状态变量 ====================== */
static CarState g_car = {0};
static uint8_t  g_inited = 0;

/* ====================== 内部工具函数 ====================== */

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

/* ====================== 1. 感知组 ====================== */

/* 从编码器 + IMU 拼装小车当前状态
 *
 * WHEELTEC 数据源:
 *   robot.MOTOR_A.Encoder / MOTOR_B.Encoder — 轮速 (m/s)
 *   imu.gyro.z — Z轴角速度 (deg/s, 需转换为 rad/s)
 */
static CarState get_car_state(void)
{
    float left_ms   = robot.MOTOR_A.Encoder;
    float right_ms  = robot.MOTOR_B.Encoder;
    float v_ms      = (left_ms + right_ms) * 0.5f;
    float omega_rad = (float)imu.gyro.z * DEG_TO_RAD;

    g_car.theta += omega_rad * APF_DT_S;
    g_car.x     += v_ms * cosf(g_car.theta) * APF_DT_S;
    g_car.y     += v_ms * sinf(g_car.theta) * APF_DT_S;
    g_car.v      = v_ms;
    g_car.omega  = omega_rad;
    g_car.tick  += (uint32_t)(APF_DT_S * 1000.0f);

    return g_car;
}

/* 低通滤波, 平滑位姿估计 */
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

/* ====================== 2. 控制与避障组 ====================== */

/* 目标路径点 (硬编码, 后续可接 ROS) */
static WayPoint waypoint_get_next(void)
{
    WayPoint target;
    target.x        = 0.0f;
    target.y        = 2.0f;      /* 2米前方 */
    target.theta    = 0.0f;
    target.is_valid = 1;
    return target;
}

/* 运动指令 → WHEELTEC 控制接口
 *
 * apf.c 输出: v_cmd (m/s), omega_cmd (rad/s)
 * 写入 robot_control.Vx / Vz,
 * Balance_task 的 Drive_Motor() 在下一个周期执行
 */
static void apf_cmd_to_robot(float v_cmd, float omega_cmd)
{
    v_cmd     = clampf(v_cmd,     -APF_V_MAX, APF_V_MAX);
    omega_cmd = clampf(omega_cmd, -APF_W_MAX, APF_W_MAX);

    robot_control.Vx = v_cmd;
    robot_control.Vz = omega_cmd;
}

/* ====================== 3. 通信组 ====================== */

/* CAN 上报小车状态 + 障碍物 */
static void ros_send_car_state(CarState state, SensorObs obs)
{
    u8 frame[8];

    /* 帧 type=0x02: 里程数据 (x, y, theta, v) */
    frame[0] = 0x02;
    pack_s16(frame, 1, clamp_to_s16(state.x * 10.0f, -30000.0f, 30000.0f));
    pack_s16(frame, 3, clamp_to_s16(state.y * 10.0f, -30000.0f, 30000.0f));
    pack_s16(frame, 5, clamp_to_s16(state.theta * 1000.0f, -30000.0f, 30000.0f));
    frame[7] = (u8)clampf(state.v + 128.0f, 0.0f, 255.0f);
    CAN1_Send_Num(0x100, frame);

    /* 帧 type=0x03: 障碍物 (仅有效时发送) */
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

/* ====================== 4. 显示组 ====================== */

/* OLED 显示 APF 信息 (降频到 ~5Hz) */
static void update_oled(CarState state, WayPoint target, SensorObs obs)
{
    static u8 div = 0;

    if (++div < 10) return;
    div = 0;

    OLED_ShowString(0, 0, (const u8 *)"APF");
    OLED_ShowNumber(32, 0, (u32)(target.is_valid), 1, 12);
    OLED_ShowNumber(0, 10, (u32)fabsf(state.x * 100.0f), 4, 12);
    OLED_ShowNumber(48, 10, (u32)fabsf(state.y * 100.0f), 4, 12);
    OLED_ShowNumber(0, 20, (u32)fabsf(robot_control.Vx * 100.0f), 4, 12);
    OLED_ShowNumber(48, 20, (u32)fabsf(robot_control.Vz * 100.0f), 4, 12);
    OLED_ShowNumber(0, 30, (u32)(obs.valid), 1, 12);
    OLED_ShowNumber(24, 30, (u32)fabsf(obs.x * 100.0f), 3, 12);
    OLED_ShowNumber(64, 30, (u32)fabsf(obs.y * 100.0f), 3, 12);
}

/* ====================== APF 主任务 (50Hz) ====================== */

void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(APF_TASK_RATE));

        /* ---- 1. 感知 ---- */
        float dist_front = g_sensor_dist_front;
        float dist_left  = g_sensor_dist_left;
        float dist_right = g_sensor_dist_right;

        CarState car_raw  = get_car_state();
        CarState car_filt = filter_car_state(car_raw);
        g_inited = 1;

        /* ---- 2. 控制与避障 ---- */
        WayPoint target = waypoint_get_next();

        APF_SensorObs sob = APF_SensorFusion(dist_front, dist_left, dist_right);

        /* 转换供通信/显示用 */
        SensorObs obs;
        obs.x          = sob.x;
        obs.y          = sob.y;
        obs.confidence = sob.confidence;
        obs.valid      = (u8)sob.valid;

        float v_cmd = 0.0f, omega_cmd = 0.0f;
        if (target.is_valid) {
            APF_Car apf_car;
            apf_car.x     = car_filt.x;
            apf_car.y     = car_filt.y;
            apf_car.theta = car_filt.theta;

            APF_CircObs apf_obs[1];
            int obs_num = APF_SensorToCircObs(&sob, &apf_car, apf_obs, 1);

            APF_Vec2 apf_target;
            apf_target.x = target.x;
            apf_target.y = target.y;

            apf_follow(apf_target, &apf_car, apf_obs, obs_num,
                       &v_cmd, &omega_cmd);
        }

        apf_cmd_to_robot(v_cmd, omega_cmd);

        /* ---- 3. 通信 ---- */
        ros_send_car_state(car_filt, obs);

        /* ---- 4. 显示 ---- */
        update_oled(car_filt, target, obs);
    }
}
