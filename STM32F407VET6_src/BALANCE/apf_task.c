/**
 * apf_task.c — APF + Stanley 循迹避障 FreeRTOS 任务（50Hz）
 *
 * 数据流：
 *   三路测距(s21c_board.rangerA/B/C) ──┐
 *   编码器里程计(robot.MOTOR_A/B.Encoder) ─┤→ Stanley循迹 + APF避障 → robot_control.Vx/Vz
 *                                        └→ balance_task 的 Drive_Motor 直接执行
 *
 * 前提：
 *   - ControlMode 保持 0（不设 PS2/APP 模式），balance_task 走 else 分支直接
 *     Drive_Motor(robot_control.Vx, Vz)。故测试时不要同时下发 RK3576 的 cmd_vel。
 *   - 参考路径为 stanley.c 里硬编码的逆时针圆（R=0.6m，后续可改由 RK3576 下发路点）。
 */

#include "apf_task.h"
#include "apf.h"
#include "stanley.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float min3(float a, float b, float c)
{
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    const float dt = 1.0f / (float)APF_TASK_RATE;   /* 0.02s */

    APF_Car car;
    car.x = 0.0f;
    car.y = 0.0f;
    car.theta = 0.0f;
    car.v = 0.0f;

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(APF_TASK_RATE));

        /* ---- 1. 三路测距（米，5.0=无遮挡）---- */
        float d_front = s21c_board.rangerA;   /* 前方 0°   */
        float d_left  = s21c_board.rangerB;   /* 左前 +45° */
        float d_right = s21c_board.rangerC;   /* 右前 -45° */

        /* ---- 2. 编码器里程计：轮速 → v/omega → 积分位姿 ---- */
        float v_left  = robot.MOTOR_A.Encoder;   /* 左轮 m/s */
        float v_right = robot.MOTOR_B.Encoder;   /* 右轮 m/s */
        float v       = (v_left + v_right) * 0.5f;
        float omega   = (v_right - v_left) / APF_WHEEL_BASE;   /* 正=左转 */

        car.theta += omega * dt;
        car.x     += v * cosf(car.theta) * dt;
        car.y     += v * sinf(car.theta) * dt;
        car.v      = v;

        /* ---- 3. Stanley 循迹 + APF 避障（转向叠加）---- */
        float omega_cmd = stanley_steering(&car) + apf_repulsive(d_front, d_left, d_right);

        /* ---- 4. 速度：遇障减速 ---- */
        float v_cmd = apf_speed_limit(min3(d_front, d_left, d_right));

        /* ---- 5. 写控制（ControlMode=0 → balance_task 直接 Drive_Motor(Vx,Vz)）---- */
        robot_control.Vx = clampf(v_cmd, -APF_V_MAX, APF_V_MAX);
        robot_control.Vz = clampf(omega_cmd, -APF_W_MAX, APF_W_MAX);
        robot_control.ControlMode = 0;
    }
}
