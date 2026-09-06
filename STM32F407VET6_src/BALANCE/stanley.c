#include "stanley.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float wrap_pi(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

float stanley_steering(const APF_Car *car)
{
    /* 1. 车相对圆心的位置 */
    float dx = car->x - STANLEY_CIRCLE_CX;
    float dy = car->y - STANLEY_CIRCLE_CY;
    float r  = sqrtf(dx * dx + dy * dy);
    if (r < 1e-4f) r = 1e-4f;

    /* 2. 横向误差：正=圆外，负=圆内 */
    float cte = r - STANLEY_CIRCLE_R;

    /* 3. 逆时针圆在最近点处的切线方向（垂直于半径向量 (dx,dy)，逆时针切线 = (-dy,dx)） */
    float tangent = atan2f(-dy, dx);

    /* 4. 航向误差（归一化到 [-pi, pi]） */
    float psi_e = wrap_pi(tangent - car->theta);

    /* 5. Stanley：航向误差 + 前视横向修正（atan2 平滑有界） */
    float v = (car->v > 0.15f) ? car->v : 0.15f;
    float omega = STANLEY_K_PSI * psi_e + STANLEY_K_CTE * atan2f(cte, v);

    return omega;
}
