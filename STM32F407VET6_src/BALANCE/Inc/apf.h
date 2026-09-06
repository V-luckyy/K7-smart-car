#ifndef __APF_H
#define __APF_H

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
#define APF_RHO_0      0.5f    /* 斥力作用半径 (m)：障碍在此范围内才斥 */
#define APF_SAFETY_DIST 0.15f  /* 安全距离 (m)                        */
#define APF_W_MAX      1.5f    /* 最大角速度限幅 (rad/s)              */
#define APF_V_MAX      0.3f    /* 最大线速度 (m/s)                    */
#define APF_V_MIN      0.13f   /* 最小线速度（保证差速内侧轮不反转）   */

#define APF_WHEEL_BASE 0.329f  /* 轮距 (m)，编码器差分运动学用（C50X 实测） */

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

#endif /* __APF_H */
