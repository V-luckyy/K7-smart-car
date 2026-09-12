#ifndef __STANLEY_H
#define __STANLEY_H

#include "apf.h"

/*
 * Reference path:
 *   y = 0
 *   heading = 0 rad
 *   direction = +x
 *   endpoint x = STANLEY_LINE_LEN
 */

/*
 * Standard Stanley gain.
 *
 * The standard Stanley steering equation is:
 *
 *   delta = psi_e + atan2(k * cte, v)
 *
 * where:
 *   psi_e: heading error
 *   cte: signed cross-track error of the front control point
 *   v: forward velocity
 */
#define STANLEY_K                    0.60f

/*
 * Distance from the robot reference point to the Stanley control point.
 * A front control point helps correct heading error before the robot
 * center crosses the reference line.
 */
#define STANLEY_CONTROL_DISTANCE     0.25f

/*
 * Speed floor used in atan2(k * cte, v).
 * This prevents excessive correction when the commanded speed is small.
 */
#define STANLEY_SPEED_MIN             0.05f

/*
 * Equivalent steering angle limit used before converting to differential
 * drive angular velocity.
 */
#define STANLEY_DELTA_MAX             0.70f

/*
 * Differential-drive robots do not have a physical front steering axle.
 * This virtual wheelbase converts the Stanley equivalent steering angle
 * into angular velocity:
 *
 *   omega = v * tan(delta) / STANLEY_VIRTUAL_WHEELBASE
 *
 * It is a tuning parameter, not the differential-drive track width.
 */
#define STANLEY_VIRTUAL_WHEELBASE     0.30f

#define STANLEY_LINE_LEN              3.0f
#define STANLEY_LINE_HEADING          0.0f

/*
 * Calculate Stanley angular velocity command.
 *
 * v_forward is the forward speed that will actually be used as the
 * base command before final differential-drive wheel-speed limiting.
 *
 * Return value:
 *   angular velocity in rad/s
 *   positive means turning left
 */
float stanley_steering(const APF_Car *car, float v_forward);

#endif