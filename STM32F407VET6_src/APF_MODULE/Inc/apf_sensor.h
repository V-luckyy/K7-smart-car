/*
 * apf_sensor.h — 三路测距传感器数据融合模块
 *
 * 将三路测距传感器（前方0°、左前+45°、右前-45°）的距离读数
 * 融合为一个虚拟障碍物坐标（车身坐标系），再转换到世界坐标系
 *
 * 车身坐标系: 车头方向 +X, 车左侧 +Y（右手系）
 */

#ifndef __APF_SENSOR_H
#define __APF_SENSOR_H

#include "apf.h"

/* ---- 传感器物理安装位置（车身坐标系, 单位: m）---- */
#define SENSOR_FRONT_X   0.12f
#define SENSOR_FRONT_Y   0.00f
#define SENSOR_LEFT_X    0.08f
#define SENSOR_LEFT_Y    0.06f
#define SENSOR_RIGHT_X   0.08f
#define SENSOR_RIGHT_Y  -0.06f

/* ---- 波束角预计算值 ---- */
#define COS_0    1.0f
#define SIN_0    0.0f
#define COS_45   0.70710678f
#define SIN_45   0.70710678f
#define COS_N45  0.70710678f
#define SIN_N45 -0.70710678f

/* ---- 传感器参数 ---- */
#define SENSOR_MAX_RANGE     2.0f    /* 最大有效量程(m) */

/* ---- 传感器融合结果 ---- */
typedef struct {
    float x, y;         /* 虚拟障碍物坐标 (车身坐标系, m) */
    float confidence;   /* 置信度 [0, 1] */
    int   valid;        /* 1=有效检测, 0=无检测 */
} APF_SensorObs;

/* ---- 函数声明 ---- */

/* 三路传感器数据融合 → 车身坐标系障碍物 */
APF_SensorObs APF_SensorFusion(float dist_front,
                                float dist_left,
                                float dist_right);

/* 车身坐标系 → 世界坐标系圆形障碍物 */
int APF_SensorToCircObs(const APF_SensorObs *obs,
                         const APF_Car *car,
                         APF_CircObs *out,
                         int max_obs);

#endif
