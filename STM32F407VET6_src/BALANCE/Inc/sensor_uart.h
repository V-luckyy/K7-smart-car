#ifndef __SENSOR_UART_H
#define __SENSOR_UART_H

#include "system.h"

/* ========================================================================
 * 三路测距传感器 UART 驱动
 *
 * 硬件分配:
 *   USART2  PA2(RX)  PA3(TX)   — 前方传感器
 *   USART3  PC11(RX) PC10(TX)  — 左侧传感器 (partial remap, 原App口)
 *   UART5   PD2(RX)  PC12(TX)  — 右侧传感器 (原蓝牙口)
 *
 * 传感器型号: TBD (如 TFmini / VL53L0X-UART)
 * 假设传感器持续主动发送距离帧, STM32仅接收, 不主动查询
 * ======================================================================== */

/* 传感器原始数据 (各ISR中填充) */
extern volatile float g_sensor_dist_front;  // cm
extern volatile float g_sensor_dist_left;
extern volatile float g_sensor_dist_right;
extern volatile uint8_t g_sensor_fresh;     // 三路都有新数据时置位

void sensor_uart_init(uint32_t baud);

#endif
