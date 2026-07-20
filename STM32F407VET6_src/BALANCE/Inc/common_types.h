#ifndef __COMMON_TYPES_H
#define __COMMON_TYPES_H

#include "system.h"

/* ========================================================================
 * 共享数据类型 — 被 main.c / apf.c / sensor_fusion.c 共同引用
 *
 * 坐标系: 小车坐标系, +x=右 +y=前, 原点=小车中心
 * 单位: m (适配 WHEELTEC 项目)
 * ======================================================================== */

/** 小车全局状态 (里程坐标系) */
typedef struct {
    float x;          // 里程计X坐标 (m)
    float y;          // 里程计Y坐标 (m)
    float theta;      // 偏航角 (rad, 从+x轴逆时针)
    float v;          // 线速度 (m/s)
    float omega;      // 角速度 (rad/s)
    uint32_t tick;    // 时间戳 (ms)
} CarState;

/** 目标路径点 (由ROS/A*规划下发) */
typedef struct {
    float x;          // 目标X (m)
    float y;          // 目标Y (m)
    float theta;      // 目标偏航角 (rad, 可选, APF不一定用)
    uint8_t is_valid; // 1=有效目标点
} WayPoint;

/** 传感器融合结果 (来自 APF_SensorFusion) */
typedef struct {
    float x;          // 虚拟障碍物X (m)
    float y;          // 虚拟障碍物Y (m)
    float confidence; // 置信度 [0, 1]
    uint8_t valid;    // 1=有效检测
} SensorObs;

#endif
