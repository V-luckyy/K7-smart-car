#include "common_types.h"
#ifndef __APF_H
#define __APF_H

/* ============================================================
 *  APF 参数宏  —— 可根据实际场地调整
 *  单位: m (适配 WHEELTEC 项目)
 * ============================================================ */
#define APF_DT           0.01f   /* 控制周期(s)，与 TIM1 的 10ms 中断对应     */
#define APF_K_ATT        1.0f    /* 引力增益：越大越快冲向目标                 */
#define APF_K_REP        2.0f    /* 斥力增益：越大绕障越猛                     */
#define APF_RHO_0        0.5f    /* 斥力感知半径(m)：障碍物在此范围内才产生斥力*/
#define APF_SAFETY_DIST  0.15f   /* 安全距离(m)，小车与障碍点至少保持此距离   */
#define APF_W_MAX        1.5f    /* 最大角速度限幅(rad/s)                      */
#define APF_V_MAX        0.3f    /* 最大线速度(m/s)                            */
#define APF_GOAL_THRESH  0.05f   /* 到达终点的判定距离阈值(m)                  */
#define APF_MAX_OBS      8

#define WHEEL_BASE_HALF  0.08f   /* 实测左右轮中心距的一半(m)，例如轮距16cm则填0.08 */

/* ========================================================================
 * APF (Artificial Potential Field) 人工势场法
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
    float theta;   /* 当前朝向角(rad)，0=正右方，逆时针为正 */
} APF_Car;

typedef struct {
    int arrived;   /* 1=已到达目标点，0=未到达 */
} APF_Result;

/* ============================================================
 *  函数声明
 * ============================================================ */

/*
 * apf_follow() — 核心接口，每个控制周期调用一次
 *
 * 参数:
 *   target    — 目标点 (x, y)，单位 m
 *   car       — 当前小车状态
 *   obs       — 圆形障碍物数组
 *   obs_num   — 障碍物数量
 *   v_cmd     — [输出] 线速度指令(m/s)
 *   omega_cmd — [输出] 角速度指令(rad/s)
 *
 * 返回:
 *   APF_Result.arrived — 1=到达目标，0=未到达
 */
APF_Result apf_follow(APF_Vec2 target,
                      APF_Car *car,
                      APF_CircObs *obs, int obs_num,
                      float *v_cmd, float *omega_cmd);

/*
 * APF_UpdateCarByEncoder() — 里程计：编码器更新小车位置
 */
void APF_UpdateCarByEncoder(APF_Car *car,
                            int delta_pulse,
                            float pulse_per_m,
                            float delta_theta);

/*
 * APF_Init() — 初始化小车状态
 */
void APF_Init(APF_Car *car, float start_x, float start_y, float start_theta);

#endif /* __APF_H */
