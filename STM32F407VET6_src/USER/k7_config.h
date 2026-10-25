#ifndef K7_CONFIG_H
#define K7_CONFIG_H

/* Active firmware profile. Historical algorithms remain compiled but unscheduled. */
#define K7_ROS_ONLY 1
#define K7_COMMAND_TIMEOUT_TICKS 30U /* Balance_task: 100 Hz, 300 ms */
#define K7_MAX_LINEAR_SPEED 0.35f
#define K7_MAX_ANGULAR_SPEED 1.2f

void K7_ReceiveMotion(float vx, float wz);

#endif
