#ifndef __APF_H
#define __APF_H

#include <stdint.h>

/*
 * APF obstacle avoidance module.
 *
 * Coordinate convention:
 *   x: forward
 *   y: left
 *   theta: yaw angle, 0 points along +x, counter-clockwise positive
 */

/* Car pose in odometry frame. */
typedef struct
{
    float x;
    float y;
    float theta;
    float v;
} APF_Car;

/* APF parameters. */
#define APF_K_REP          2.0f
#define APF_RHO_0          0.9f
#define APF_SAFETY_DIST    0.15f

#define APF_V_MAX          0.30f
#define APF_V_MIN          0.13f
#define APF_W_MAX          1.50f

/*
 * Differential-drive track width.
 * This must match TOP_DIFF_wheelspacing in robot_select_init.h.
 */
#define APF_TRACK_WIDTH    0.329f

/*
 * Maximum absolute speed of the faster differential-drive wheel.
 * The final Vx/Vz command is scaled together if this limit is exceeded.
 */
#define APF_WHEEL_SPEED_MAX    APF_V_MAX

/*
 * When the front sensor detects an obstacle but the two side sensors
 * have not detected it yet, steer toward the side with more free space.
 */
#define APF_FRONT_STEER_EN       1
#define APF_FRONT_STEER_MAX      0.60f
#define APF_FRONT_SIDE_DEAD      0.12f
#define APF_FRONT_DEFAULT_DIR    1

/* Sensor directions in the vehicle frame. */
#define SENSOR_ANGLE_FRONT       0.0f
#define SENSOR_ANGLE_LEFT        0.7853981634f
#define SENSOR_ANGLE_RIGHT      -0.7853981634f

/*
 * Returns obstacle avoidance angular velocity in rad/s.
 *
 * Input distances are in meters.
 * Positive output means turning left.
 */
float apf_repulsive(float dist_front,
                    float dist_left,
                    float dist_right);

/*
 * Returns the obstacle-limited forward speed in m/s.
 */
float apf_speed_limit(float dist_min);

/*
 * Debug data periodically sent by data_task through the 0xFB frame.
 */
typedef struct
{
    float x;
    float y;
    float theta;

    float v_cmd;
    float w_cmd;
    float w_stanley;
    float w_apf;

    float v_act;
    float w_act;

    float dA;
    float dB;
    float dC;
} APF_Debug_t;

extern APF_Debug_t g_apf_debug;
extern uint8_t g_apf_debug_valid;

#endif