#include "stanley.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

static float wrap_pi(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0f * M_PI;
    }

    return angle;
}

float stanley_steering(const APF_Car *car, float v_forward)
{
    float control_y;
    float cte;
    float psi_e;
    float v_eff;
    float delta;
    float omega;

    if (car == 0)
    {
        return 0.0f;
    }

    /*
     * Stanley control point:
     *
     *   x_control = x + L * cos(theta)
     *   y_control = y + L * sin(theta)
     *
     * For the current straight path y = 0, only y_control is required.
     */
    control_y = car->y +
                STANLEY_CONTROL_DISTANCE * sinf(car->theta);

    /*
     * Signed cross-track error.
     *
     * Positive cte means the control point is to the right of the path,
     * so the resulting steering correction is positive/left.
     *
     * For this coordinate convention:
     *   control point left of path  -> cte < 0 -> right correction
     *   control point right of path -> cte > 0 -> left correction
     */
    cte = -control_y;

    /*
     * Reference heading is +x. The error is always wrapped to [-pi, pi].
     */
    psi_e = wrap_pi(STANLEY_LINE_HEADING - car->theta);

    /*
     * Use the planned forward speed rather than stale measured speed.
     * This makes the speed and steering commands generated in the same
     * control cycle consistent with each other.
     */
    v_eff = fabsf(v_forward);

    if (v_eff < STANLEY_SPEED_MIN)
    {
        v_eff = STANLEY_SPEED_MIN;
    }

    /*
     * Standard Stanley equivalent steering angle:
     *
     *   delta = psi_e + atan2(k * cte, v)
     */
    delta = psi_e + atan2f(STANLEY_K * cte, v_eff);
    delta = clampf(delta,
                   -STANLEY_DELTA_MAX,
                    STANLEY_DELTA_MAX);

    /*
     * Convert the equivalent steering angle to angular velocity for a
     * differential-drive vehicle.
     */
    omega = v_forward *
            tanf(delta) /
            STANLEY_VIRTUAL_WHEELBASE;

    return omega;
}