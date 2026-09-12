/*
 * APF + Stanley differential-drive controller.
 *
 * Data flow:
 *
 *   GY-53 distances
 *       |
 *       +--> APF obstacle angular velocity
 *
 *   encoder wheel velocities
 *       |
 *       +--> odometry x/y/theta
 *       |
 *       +--> Stanley angular velocity
 *
 *   obstacle-limited forward speed
 *       |
 *       +--> joint Vx/Vz limiting
 *       |
 *       +--> robot_control.Vx/Vz
 *       |
 *       +--> Balance_task -> differential-drive inverse kinematics
 */

#include "apf_task.h"
#include "apf.h"
#include "stanley.h"
#include "balance_task.h"

#include <math.h>

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

static float min3(float a, float b, float c)
{
    float minimum = (a < b) ? a : b;

    return (minimum < c) ? minimum : c;
}

/*
 * Limit the differential-drive command jointly.
 *
 * Differential-drive wheel speeds are:
 *
 *   v_left  = Vx - omega * track_width / 2
 *   v_right = Vx + omega * track_width / 2
 *
 * The first limit guarantees that the inner wheel does not reverse:
 *
 *   |omega| <= 2 * Vx / track_width
 *
 * The second limit keeps the faster outer wheel below the configured
 * maximum speed. Vx and omega are scaled together so that the curvature
 * is preserved.
 */
static void limit_diff_drive_command(float *v_cmd, float *omega_cmd)
{
    float omega_limit;
    float outer_wheel_speed;
    float scale;

    if (v_cmd == 0 || omega_cmd == 0)
    {
        return;
    }

    if (*v_cmd <= 0.0f)
    {
        *v_cmd = 0.0f;
        *omega_cmd = 0.0f;
        return;
    }

    /*
     * Keep both wheels moving forward:
     *
     *   Vx - |Vz| * track_width / 2 >= 0
     */
    omega_limit = 2.0f * (*v_cmd) / APF_TRACK_WIDTH;

    if (omega_limit > APF_W_MAX)
    {
        omega_limit = APF_W_MAX;
    }

    *omega_cmd = clampf(*omega_cmd,
                        -omega_limit,
                         omega_limit);

    /*
     * With the inner wheel constrained to be non-negative, the faster
     * wheel speed is Vx + |Vz| * track_width / 2.
     */
    outer_wheel_speed = (*v_cmd) +
                        fabsf(*omega_cmd) *
                        APF_TRACK_WIDTH *
                        0.5f;

    if (outer_wheel_speed > APF_WHEEL_SPEED_MAX &&
        outer_wheel_speed > 0.0f)
    {
        scale = APF_WHEEL_SPEED_MAX / outer_wheel_speed;

        *v_cmd *= scale;
        *omega_cmd *= scale;
    }
}

APF_Debug_t g_apf_debug;
uint8_t g_apf_debug_valid = 0;

void APF_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    const float dt = 1.0f / (float)APF_TASK_RATE;

    APF_Car car;

    int done = 0;

    (void)pvParameters;

    car.x = 0.0f;
    car.y = 0.0f;
    car.theta = 0.0f;
    car.v = 0.0f;

    while (1)
    {
        float d_front;
        float d_left;
        float d_right;

        float v_left;
        float v_right;
        float v_act;
        float omega_act;

        float omega_stanley;
        float omega_apf;
        float omega_cmd;

        float v_cmd;
        float v_out;
        float w_out;

        vTaskDelayUntil(&lastWakeTime, F2T(APF_TASK_RATE));

        /*
         * Do not control or integrate odometry during the chassis
         * self-check. Balance_task drives the motors during this period.
         * This prevents self-check motion from shifting the APF origin.
         */
        if (robot_check.check_end == 0)
        {
            robot_control.Vx = 0.0f;
            robot_control.Vy = 0.0f;
            robot_control.Vz = 0.0f;
            robot_control.ControlMode = 0;

            car.v = 0.0f;

            continue;
        }

        /*
         * Three GY-53 distance sensors.
         */
        d_front = s21c_board.rangerA;
        d_left  = s21c_board.rangerB;
        d_right = s21c_board.rangerC;

        /*
         * Encoder odometry.
         *
         * MOTOR_A is the left wheel.
         * MOTOR_B is the right wheel.
         */
        v_left  = robot.MOTOR_A.Encoder;
        v_right = robot.MOTOR_B.Encoder;

        v_act = (v_left + v_right) * 0.5f;

        omega_act = (v_right - v_left) /
                    APF_TRACK_WIDTH;

        car.theta += omega_act * dt;
        car.x += v_act * cosf(car.theta) * dt;
        car.y += v_act * sinf(car.theta) * dt;
        car.v = v_act;

        /*
         * The endpoint is determined by x coordinate, not accumulated
         * travel distance. This remains correct when the robot takes a
         * lateral detour around an obstacle.
         */
        if (!done && car.x >= STANLEY_LINE_LEN)
        {
            done = 1;
        }

        /*
         * First reduce forward speed according to the closest obstacle.
         * Stanley uses this same speed for its current-cycle calculation.
         */
        v_cmd = apf_speed_limit(min3(d_front,
                                     d_left,
                                     d_right));

        omega_stanley = stanley_steering(&car, v_cmd);
        omega_apf = apf_repulsive(d_front,
                                  d_left,
                                  d_right);

        /*
         * Both steering components are combined before the final
         * differential-drive feasibility limits.
         */
        omega_cmd = omega_stanley + omega_apf;

        if (done)
        {
            v_cmd = 0.0f;
            omega_cmd = 0.0f;
        }

        v_out = clampf(v_cmd,
                       0.0f,
                       APF_V_MAX);

        w_out = clampf(omega_cmd,
                       -APF_W_MAX,
                        APF_W_MAX);

        /*
         * Vx and Vz are constrained together. The resulting command is
         * still sent as simultaneous linear and angular velocity.
         */
        limit_diff_drive_command(&v_out, &w_out);

        robot_control.Vx = v_out;
        robot_control.Vy = 0.0f;
        robot_control.Vz = w_out;
        robot_control.ControlMode = 0;

        /*
         * Debug data.
         *
         * w_stanley and w_apf are the controller components before the
         * final joint differential-drive limiting.
         * w_cmd is the actual final angular command.
         */
        g_apf_debug.x = car.x;
        g_apf_debug.y = car.y;
        g_apf_debug.theta = car.theta;

        g_apf_debug.v_cmd = v_out;
        g_apf_debug.w_cmd = w_out;
        g_apf_debug.w_stanley = omega_stanley;
        g_apf_debug.w_apf = omega_apf;

        g_apf_debug.v_act = v_act;
        g_apf_debug.w_act = omega_act;

        g_apf_debug.dA = d_front;
        g_apf_debug.dB = d_left;
        g_apf_debug.dC = d_right;

        g_apf_debug_valid = 1;
    }
}