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
 *   - 参考路径为 stanley.c 里硬编码的直线段（沿 +x 走 STANLEY_LINE_LEN=3m，到点停车；
 *     后续可改由 RK3576 下发路点）。
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

/* ---- 调参调试数据（0xFB 上行帧数据源，见 apf.h）---- */
APF_Debug_t  g_apf_debug;
uint8_t      g_apf_debug_valid = 0;

void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    const float dt = 1.0f / (float)APF_TASK_RATE;   /* 0.02s */

    APF_Car car;
    car.x = 0.0f;
    car.y = 0.0f;
    car.theta = 0.0f;
    car.v = 0.0f;

    float dist = 0.0f;   /* 已行驶里程 (m)，直线行驶到 STANLEY_LINE_LEN 后停车 */
    int   done  = 0;     /* 1=已走完参考直线段，停车 */

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

        /* ---- 2b. 里程累计：沿直线走到 STANLEY_LINE_LEN(m) 后置 done 停车 ---- */
        if (!done)
        {
            dist += v * dt;
            if (dist >= STANLEY_LINE_LEN) done = 1;
        }

        /* ---- 3. Stanley 循迹 + APF 避障（转向叠加，分量分开保存供调试）---- */
        float omega_stanley = stanley_steering(&car);
        float omega_apf     = apf_repulsive(d_front, d_left, d_right);
        float omega_cmd     = omega_stanley + omega_apf;

        /* ---- 4. 速度：遇障减速 ---- */
        float v_cmd = apf_speed_limit(min3(d_front, d_left, d_right));

        /* 已到终点：清掉一切指令（原地停住，不再循迹/避障） */
        if (done) { v_cmd = 0.0f; omega_cmd = 0.0f; }

        /* ---- 5. 写控制（ControlMode=0 → balance_task 直接 Drive_Motor(Vx,Vz)）---- */
        float v_out = clampf(v_cmd, -APF_V_MAX, APF_V_MAX);
        float w_out = clampf(omega_cmd, -APF_W_MAX, APF_W_MAX);
        robot_control.Vx = v_out;
        robot_control.Vz = w_out;
        robot_control.ControlMode = 0;

        /* ---- 6. 调试数据采样（50Hz，data_task 每 20Hz 组装 0xFB 帧上行）---- */
        g_apf_debug.x        = car.x;
        g_apf_debug.y        = car.y;
        g_apf_debug.theta    = car.theta;
        g_apf_debug.v_cmd    = v_out;
        g_apf_debug.w_cmd    = w_out;
        g_apf_debug.w_stanley = omega_stanley;
        g_apf_debug.w_apf    = omega_apf;
        g_apf_debug.v_act    = v;
        g_apf_debug.w_act    = omega;
        g_apf_debug.dA       = d_front;
        g_apf_debug.dB       = d_left;
        g_apf_debug.dC       = d_right;
        g_apf_debug_valid    = 1;
    }
}
