#ifndef __STANLEY_H
#define __STANLEY_H

#include "apf.h"   /* 使用 APF_Car 类型 */

/* ============================================================
 * Stanley 循迹参数 —— 硬编码逆时针圆
 * 坐标约定：x=前进方向，y=左，theta=0 朝 +x，逆时针为正（与里程计一致）
 * ============================================================ */
#define STANLEY_K_PSI     1.0f    /* 航向误差增益                     */
#define STANLEY_K_CTE     1.5f    /* 横向误差增益（前视修正）         */

#define STANLEY_CIRCLE_R   0.6f   /* 圆半径 (m)                        */
#define STANLEY_CIRCLE_CX  0.0f   /* 圆心 X (m)                        */
#define STANLEY_CIRCLE_CY  0.0f   /* 圆心 Y (m)                        */

/*
 * stanley_steering() — 循迹转向角速度
 * 参数: car — 小车当前位姿（里程计系）
 * 返回: omega_cmd (rad/s)，正=左转（逆时针）
 *
 * 原理：航向误差（圆切线方向 vs 车头）+ 横向误差（车到圆心距离 - R）
 *       叠加得到转向角速度。正负号若反了，把 STANLEY_K_PSI/K_CTE 取负即可。
 */
float stanley_steering(const APF_Car *car);

#endif /* __STANLEY_H */
