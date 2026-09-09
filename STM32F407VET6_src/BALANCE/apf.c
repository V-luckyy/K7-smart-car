#include "apf.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 单路传感器的斥力强度：d < RHO_0 时 >0，越近越大 */
static float rep_force(float d)
{
    if (d >= APF_RHO_0) return 0.0f;
    if (d < 0.02f) d = 0.02f;              /* 防除零 */
    float inv = 1.0f / d - 1.0f / APF_RHO_0;
    return APF_K_REP * inv / (d * d);      /* K * (1/d - 1/ρ0) / d² */
}

float apf_repulsive(float dist_front, float dist_left, float dist_right)
{
    /* 每个传感器在"障碍物方向"产生斥力，横向分量（-sin φ）决定转向：
     *   左前(+45°) → 转向右侧(负)；右前(-45°) → 转向左侧(正)
     *   正前方(0°)   → 无横向分量，只减速
     */
    float omega = 0.0f;

    omega += rep_force(dist_front) * (-sinf(SENSOR_ANGLE_FRONT));
    omega += rep_force(dist_left)  * (-sinf(SENSOR_ANGLE_LEFT));
    omega += rep_force(dist_right) * (-sinf(SENSOR_ANGLE_RIGHT));

#if APF_FRONT_STEER_EN
    /* 正前近障：上面三项在"障碍正中、两侧还没探到"时不产生转向，车会顶上去。
     * 这里按"哪边更空往哪边绕"补一个小转向（越近越强），保证能提前躲开正前方障碍。 */
    if (dist_front < APF_RHO_0)
    {
        float room = dist_left - dist_right;            /* >0 → 左边更空 */
        float dir;
        if      (room >  APF_FRONT_SIDE_DEAD) dir =  1.0f;         /* 左空 → 左绕 */
        else if (room < -APF_FRONT_SIDE_DEAD) dir = -1.0f;         /* 右空 → 右绕 */
        else    dir = (float)APF_FRONT_DEFAULT_DIR;                /* 居中 → 默认方向 */

        float k = (APF_RHO_0 - dist_front) / APF_RHO_0;           /* 越近越强 (0~1) */
        omega += dir * APF_FRONT_STEER_MAX * k;
    }
#endif /* APF_FRONT_STEER_EN */

    return clampf(omega, -APF_W_MAX, APF_W_MAX);
}

float apf_speed_limit(float dist_min)
{
    /* 越近越慢，进入安全距离后压到最低速 */
    if (dist_min >= APF_RHO_0) return APF_V_MAX;
    if (dist_min <= APF_SAFETY_DIST) return APF_V_MIN;

    float t = (dist_min - APF_SAFETY_DIST) / (APF_RHO_0 - APF_SAFETY_DIST);
    return APF_V_MIN + (APF_V_MAX - APF_V_MIN) * t;
}
