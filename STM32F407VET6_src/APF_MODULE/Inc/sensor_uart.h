#ifndef __SENSOR_UART_H
#define __SENSOR_UART_H

#include "system.h"

/* ========================================================================
 * 三路测距传感器 UART 驱动
 *
 * 硬件分配:
 *   USART2  PA2(RX)  PA3(TX)   — 前方传感器
 *   USART3  PC11(RX) PC10(TX)  — 左侧传感器
 *   UART5   PD2(RX)  PC12(TX)  — 右侧传感器
 *
 * 传感器持续主动发送距离帧, STM32仅接收, 不主动查询
 * ======================================================================== */

/* 传感器原始数据 (各ISR中填充), 单位 m */
extern volatile float g_sensor_dist_front;
extern volatile float g_sensor_dist_left;
extern volatile float g_sensor_dist_right;
extern volatile uint8_t g_sensor_fresh;

void sensor_uart_init(uint32_t baud);

#endif
