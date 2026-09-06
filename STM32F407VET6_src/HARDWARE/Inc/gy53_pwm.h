#ifndef __GY53_PWM_H
#define __GY53_PWM_H

#include "sys.h"

/* GY-53 VL53L0X PWM ranging input.
 * C50C P5 reserved GPIO header default wiring:
 * front -> PE5, left -> PE7, right -> PE8.
 * GY-53 PWM formula: distance(mm) = high_time(us) / 10.
 */
#define GY53_PWM_SENSOR_COUNT       3
#define GY53_PWM_FRONT              0
#define GY53_PWM_LEFT               1
#define GY53_PWM_RIGHT              2

#define GY53_PWM_FRONT_PORT         GPIOE
#define GY53_PWM_FRONT_PIN          GPIO_Pin_5
#define GY53_PWM_FRONT_EXTI_LINE    EXTI_Line5
#define GY53_PWM_FRONT_PIN_SOURCE   EXTI_PinSource5

#define GY53_PWM_LEFT_PORT          GPIOE
#define GY53_PWM_LEFT_PIN           GPIO_Pin_7
#define GY53_PWM_LEFT_EXTI_LINE     EXTI_Line7
#define GY53_PWM_LEFT_PIN_SOURCE    EXTI_PinSource7

#define GY53_PWM_RIGHT_PORT         GPIOE
#define GY53_PWM_RIGHT_PIN          GPIO_Pin_8
#define GY53_PWM_RIGHT_EXTI_LINE    EXTI_Line8
#define GY53_PWM_RIGHT_PIN_SOURCE   EXTI_PinSource8

#define GY53_PWM_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOE
#define GY53_PWM_EXTI_IRQn          EXTI9_5_IRQn
#define GY53_PWM_EXTI_PRIO          5
#define GY53_PWM_EXTI_MASK          (GY53_PWM_FRONT_EXTI_LINE | GY53_PWM_LEFT_EXTI_LINE | GY53_PWM_RIGHT_EXTI_LINE)
#define GY53_PWM_CAPTURE_DELAY_MS   300U

#define GY53_PWM_TASK_PRIO          2
#define GY53_PWM_STK_SIZE           128
#define GY53_PWM_TASK_RATE          RATE_10_HZ

#define GY53_PWM_MIN_HIGH_US        20U
#define GY53_PWM_MAX_HIGH_US        40000U
#define GY53_PWM_ONLINE_TIMEOUT_MS  500U

typedef struct
{
	volatile u16 distance_mm;
	volatile u32 high_us;
	volatile u32 last_pulse_us;
	volatile u32 edge_count;
	volatile u32 invalid_count;
	volatile u8 online;
	volatile u8 updated;
	volatile u32 update_count;
	volatile u32 last_update_tick;
}GY53_PWM_SENSOR_t;

void GY53_PWM_Init(void);
void GY53_PWM_EnableCapture(void);
void GY53_PWM_task(void *pvParameters);
void GY53_PWM_EXTI_IRQHandler(void);

extern GY53_PWM_SENSOR_t gy53_pwm_sensor[GY53_PWM_SENSOR_COUNT];
extern volatile u8 gy53_pwm_capture_enabled;

#endif
