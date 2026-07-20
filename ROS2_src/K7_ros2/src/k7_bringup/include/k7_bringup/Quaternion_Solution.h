// 原样复制自 wheeltec_ros2 turn_on_wheeltec_robot (ROS2 Humble)，仅 include 路径改为 k7_bringup/k7_robot.h

#ifndef __QUATERNION_SOLUTION_H_
#define __QUATERNION_SOLUTION_H_
#include "k7_bringup/k7_robot.h"
float InvSqrt(float number);
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);

#endif

