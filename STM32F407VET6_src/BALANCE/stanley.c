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
    /* 1. 航向误差：参考方向 = +x（0 rad），psi_e = 0 - theta（theta>0 为左偏，需右回） */
    float psi_e = wrap_pi(STANLEY_LINE_HEADING - car->theta);

    /* 2. 横向误差 cte：到直线 y=0 的有向距离。
     *    约定：车偏左(+y，即 theta 为正时前进会偏向的那一侧)取负，
     *    使 cte>0 时指令左转有收敛性。若实车方向反了，把 K_PSI/K_CTE 取负。 */
    float cte = -car->y;

    /* 3. Stanley：航向误差 + 前视横向修正（atan2 平滑有界） */
    float v = (car->v > 0.15f) ? car->v : 0.15f;
    float omega = STANLEY_K_PSI * psi_e + STANLEY_K_CTE * atan2f(cte, v);

    return omega;
}
