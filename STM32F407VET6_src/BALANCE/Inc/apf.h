#ifndef __APF_H
#define __APF_H

#include <stdint.h>   /* uint8_t（apf.c 直接 include 本头，需自带） */

/* ============================================================
 * APF 人工势场法 —— 避障模块
 * 配合 stanley.c 循迹：Stanley 出循迹转向，APF 出避障转向，二者叠加。
 *
 * 坐标约定：x=前进方向，y=左，theta=0 朝 +x，逆时针为正。
 * ============================================================ */

/* ---- 小车位姿（里程计系） ---- */
typedef struct {
    float x;      /* 里程计 X (m) */
    float y;      /* 里程计 Y (m) */
    float theta;  /* 偏航角 (rad)，0=朝 +x，逆时针为正 */
    float v;      /* 当前线速度 (m/s) */
} APF_Car;

/* ---- APF 参数宏（可按场地调整） ---- */
#define APF_K_REP      2.0f    /* 斥力增益：越大绕障越猛             */
#define APF_RHO_0      0.9f    /* 斥力作用半径 (m)：障碍在此范围内才斥 */
#define APF_SAFETY_DIST 0.15f  /* 安全距离 (m)                        */
#define APF_W_MAX      1.5f    /* 最大角速度限幅 (rad/s)              */
#define APF_V_MAX      0.3f    /* 最大线速度 (m/s)                    */
#define APF_V_MIN      0.13f   /* 最小线速度（保证差速内侧轮不反转）   */

#define APF_WHEEL_BASE 0.329f  /* 轮距 (m)，编码器差分运动学用（C50X 实测） */

/* ---- "正前近障朝空侧绕"（2026-09-06）----
 * 前方(0°)传感器原本只减速不转向；当障碍正中、两侧 ±45° 还没探到时车会顶上去。
 * 此项：d_front < APF_RHO_0 时，按"左右哪边更空就往哪边绕"补一个小转向，
 * 让正前方障碍也能提前绕开；左右读数相当时用 APF_FRONT_DEFAULT_DIR 兜底方向。
 */
#define APF_FRONT_STEER_EN     1       /* 1=使能, 0=关闭（退回原行为）       */
#define APF_FRONT_STEER_MAX    0.6f    /* 该项最大转向角速度 (rad/s)         */
#define APF_FRONT_SIDE_DEAD    0.12f   /* 左右读数差(m)小于此视为"居中"      */
#define APF_FRONT_DEFAULT_DIR  1       /* 居中等距时默认绕向：1=左绕, -1=右绕 */

/* ---- 三路传感器角度（车身系，弧度） ----
 * 前方 0°，左前 +45°，右前 -45°（与 gy53_pwm.c 的 rangerA/B/C 对应） */
#define SENSOR_ANGLE_FRONT   0.0f
#define SENSOR_ANGLE_LEFT    0.7853981634f    /* +45° */
#define SENSOR_ANGLE_RIGHT  -0.7853981634f    /* -45° */

/*
 * apf_repulsive() — 三路测距 → 避障转向角速度
 * 参数:
 *   dist_front / dist_left / dist_right — 三路距离 (m)，>=APF_RHO_0 视为无碍
 * 返回:
 *   omega_rep (rad/s)，正=左转
 *
 * 原理：每路传感器在障碍物方向产生斥力，横向分量累加成转向；
 *       正前方障碍只减速不转向（由左右两路决定绕行方向）。
 */
float apf_repulsive(float dist_front, float dist_left, float dist_right);

/*
 * apf_speed_limit() — 根据最近障碍距离限制线速度（遇障减速）
 * 参数: dist_min — 三路中最近距离 (m)
 * 返回: 允许的最大线速度 (m/s)
 */
float apf_speed_limit(float dist_min);

/* ============================================================
 * APF/Stanley 调参调试数据（2026-09-06）
 * apf_task 每 50Hz 采样填入 g_apf_debug，data_task 每 20Hz 组装成
 * 0xFB 上行帧发给 K7（27B，int16 大端 ×1000，BCC 校验），K7 侧发布 /apf_debug。
 * 用途：实车调参时在 K7 侧记录"实际轨迹 vs 参考圆、控制器内部分量、距离"。
 * ============================================================ */
typedef struct {
    float x;        /* 里程计位姿 X (m) */
    float y;        /* 里程计位姿 Y (m) */
    float theta;    /* 偏航角 (rad)，未 wrap，长跑几圈会溢出 int16 刻度，绘图侧 unwrap */
    float v_cmd;    /* 下发线速度 (m/s) */
    float w_cmd;    /* 下发角速度 (rad/s) = w_stanley + w_apf */
    float w_stanley;/* 循迹转向分量 (rad/s) */
    float w_apf;    /* 避障转向分量 (rad/s) */
    float v_act;    /* 实际线速度 (m/s)，编码器 */
    float w_act;    /* 实际角速度 (rad/s)，编码器差分 */
    float dA;       /* 前方 0°  测距 (m)，5.0=无遮挡 */
    float dB;       /* 左前 +45°测距 (m) */
    float dC;       /* 右前 -45°测距 (m) */
} APF_Debug_t;

extern APF_Debug_t g_apf_debug;      /* 最新一次采样 */
extern uint8_t     g_apf_debug_valid;/* 1=APF 任务已跑过至少一次，data_task 据此发送 0xFB */

#endif /* __APF_H */
