#ifndef __STANLEY_H
#define __STANLEY_H

#include "apf.h"   /* 使用 APF_Car 类型 */

/* ============================================================
 * Stanley 循迹参数 —— 硬编码直线段：沿 +x 走 STANLEY_LINE_LEN(m)
 * 坐标约定：x=前进方向，y=左，theta=0 朝 +x，逆时针为正（与里程计一致）
 * 参考路径：直线 y=0，方向 +x，起点 (0,0)，终点 (STANLEY_LINE_LEN, 0)。
 * ============================================================ */
#define STANLEY_K_PSI     0.6f    /* 航向误差增益                     */
#define STANLEY_K_CTE     0.8f    /* 横向误差增益（前视修正）         */

#define STANLEY_LINE_LEN     3.0f   /* 直线行驶长度 (m)，里程 ≥ 此值即停车（apf_task 判断） */
#define STANLEY_LINE_HEADING 0.0f   /* 直线方向 (rad)：沿 +x */

/*
 * stanley_steering() — 循迹转向角速度（直线段）
 * 参数: car — 小车当前位姿（里程计系）
 * 返回: omega_cmd (rad/s)，正=左转
 *
 * 原理：航向误差（参考航向 0 vs 车头）+ 横向误差（到直线 y=0 的距离）
 *       叠加得到转向角速度。若实车转向方向反了，把 STANLEY_K_PSI/K_CTE 取负即可。
 */
float stanley_steering(const APF_Car *car);

#endif /* __STANLEY_H */
