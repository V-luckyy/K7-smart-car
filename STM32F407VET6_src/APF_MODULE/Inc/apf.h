#ifndef __APF_H
#define __APF_H

#include "common_types.h"

/* ============================================================
 *  APF 参数宏 — 可根据实际场地调整, 单位: m
 * ============================================================ */
#define APF_DT           0.01f   /* 控制周期(s)                         */
#define APF_K_ATT        1.0f    /* 引力增益：越大越快冲向目标           */
#define APF_K_REP        2.0f    /* 斥力增益：越大绕障越猛               */
#define APF_RHO_0        0.5f    /* 斥力感知半径(m)                      */
#define APF_SAFETY_DIST  0.15f   /* 安全距离(m)                          */
#define APF_W_MAX        1.5f    /* 最大角速度限幅(rad/s)                */
#define APF_V_MAX        0.3f    /* 最大线速度(m/s)                      */
#define APF_GOAL_THRESH  0.05f   /* 到达终点判定距离阈值(m)              */
#define APF_MAX_OBS      8

/* ========================================================================
 * APF (Artificial Potential Field) 人工势场法 — 类型定义
 * ======================================================================== */

typedef struct { float x; float y; } APF_Vec2;

typedef struct {
    float cx;
    float cy;
    float confidence;
} APF_CircObs;

typedef struct {
    float x;
    float y;
    float theta;
} APF_Car;

typedef struct {
    int arrived;
} APF_Result;

/* ============================================================
 *  函数声明
 * ============================================================ */

/* 核心接口: 人工势场法路径跟随 */
APF_Result apf_follow(APF_Vec2 target,
                      APF_Car *car,
                      APF_CircObs *obs, int obs_num,
                      float *v_cmd, float *omega_cmd);

/* 里程计: 编码器更新小车位置 */
void APF_UpdateCarByEncoder(APF_Car *car,
                            int delta_pulse,
                            float pulse_per_m,
                            float delta_theta);

/* 初始化小车状态 */
void APF_Init(APF_Car *car, float start_x, float start_y, float start_theta);

#endif
