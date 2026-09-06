#ifndef __SYSTEM_H
#define __SYSTEM_H

//���ó�����ʼƫ�Ƶ�ַ. ������ʼ��ַ = 0x8000000 + VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x00000

//оƬͷ�ļ�
#include "stm32f4xx.h"

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

//FreeRTOS related header files
// FreeRTOS���ͷ�ļ�
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//Board level support package related header files
//�弶֧�ְ�(Ӳ��)���ͷ�ļ�
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"
#include "can.h"
#include "encoder.h"
#include "key.h"
#include "LED.h"
#include "motor.h"
#include "oled.h"
#include "pstwo.h"
#include "stmflash.h"
#include "auto_recharge.h"
#include "buzzer.h"
#include "enable_key.h"
#include "uartx.h"
#include "driver_d50a.h"
#include "I2C.h"
#include "MPU6050.h"
#include "ICM20948.h"
#include "remote.h"
#include "gy53_pwm.h"
#include "usb_host.h"

/* Diagnostic build controls. Set the delay to 0 after troubleshooting. */
#define SYSTEM_DIAG_BOOT_SCREEN_MS  800U
#define SYSTEM_DIAG_TRACE_DEPTH     8U

typedef enum
{
    SYSTEM_DIAG_STAGE_RESET             = 0x0100U,
    SYSTEM_DIAG_STAGE_I2C_READY         = 0x0110U,
    SYSTEM_DIAG_STAGE_UART1_READY       = 0x0120U,
    SYSTEM_DIAG_STAGE_IMU_DETECT        = 0x0130U,
    SYSTEM_DIAG_STAGE_IMU_READY         = 0x0131U,
    SYSTEM_DIAG_STAGE_USB_START         = 0x0140U,
    SYSTEM_DIAG_STAGE_USB_READY         = 0x0141U,
    SYSTEM_DIAG_STAGE_BASIC_IO_READY    = 0x0200U,
    SYSTEM_DIAG_STAGE_GY53_START        = 0x0210U,
    SYSTEM_DIAG_STAGE_GY53_READY        = 0x0211U,
    SYSTEM_DIAG_STAGE_OLED_START        = 0x0220U,
    SYSTEM_DIAG_STAGE_OLED_READY        = 0x0221U,
    SYSTEM_DIAG_STAGE_UARTS_READY       = 0x0230U,
    SYSTEM_DIAG_STAGE_HARDWARE_READY    = 0x0240U,
    SYSTEM_DIAG_STAGE_SYSTEM_READY      = 0x02FFU,
    SYSTEM_DIAG_STAGE_START_TASK_READY  = 0x0300U,
    SYSTEM_DIAG_STAGE_SCHEDULER_START   = 0x0310U,
    SYSTEM_DIAG_STAGE_START_TASK_RUN    = 0x0320U,
    SYSTEM_DIAG_STAGE_TASKS_CREATED     = 0x0330U,
    SYSTEM_DIAG_STAGE_RTOS_RUNNING      = 0x0400U,
    SYSTEM_DIAG_STAGE_FATAL             = 0xE000U,
    SYSTEM_DIAG_STAGE_CPU_FAULT         = 0xF000U
} SYSTEM_DIAG_STAGE_t;

#define SYSTEM_DIAG_ERROR_IMU_ID             (1UL << 0)
#define SYSTEM_DIAG_ERROR_HW_VERSION         (1UL << 1)
#define SYSTEM_DIAG_ERROR_TASK_CREATE        (1UL << 2)
#define SYSTEM_DIAG_ERROR_START_TASK         (1UL << 3)
#define SYSTEM_DIAG_ERROR_SCHEDULER_RETURN   (1UL << 4)
#define SYSTEM_DIAG_ERROR_ASSERT             (1UL << 5)
#define SYSTEM_DIAG_ERROR_MALLOC             (1UL << 6)
#define SYSTEM_DIAG_ERROR_STACK_OVERFLOW     (1UL << 7)
#define SYSTEM_DIAG_ERROR_CPU_FAULT          (1UL << 8)
#define SYSTEM_DIAG_ERROR_USB_HOST           (1UL << 9)
#define SYSTEM_DIAG_ERROR_ADC_TIMEOUT        (1UL << 10)
#define SYSTEM_DIAG_ERROR_UART_TIMEOUT       (1UL << 11)

#define SYSTEM_DIAG_RUNTIME_SHOW_TASK        (1UL << 0)
#define SYSTEM_DIAG_RUNTIME_PWM_CAPTURE      (1UL << 1)

#define SYSTEM_DIAG_TASK_START               (1UL << 0)
#define SYSTEM_DIAG_TASK_BALANCE             (1UL << 1)
#define SYSTEM_DIAG_TASK_SHOW                (1UL << 2)
#define SYSTEM_DIAG_TASK_LED                 (1UL << 3)
#define SYSTEM_DIAG_TASK_DATA                (1UL << 4)
#define SYSTEM_DIAG_TASK_GY53                (1UL << 5)
#define SYSTEM_DIAG_TASK_D50A                (1UL << 6)
#define SYSTEM_DIAG_TASK_IMU                 (1UL << 7)
#define SYSTEM_DIAG_TASK_PS2                 (1UL << 8)
#define SYSTEM_DIAG_TASK_REPORT              (1UL << 9)
#define SYSTEM_DIAG_TASK_USB                 (1UL << 10)
#define SYSTEM_DIAG_TASK_APF                 (1UL << 11)

typedef struct
{
    volatile u32 magic;
    volatile u32 reset_csr;
    volatile u32 stage;
    volatile u32 previous_stage;
    volatile u32 trace_index;
    volatile u32 stage_trace[SYSTEM_DIAG_TRACE_DEPTH];
    volatile u32 error_flags;
    volatile u32 runtime_flags;
    volatile u32 task_ok_mask;
    volatile u32 task_fail_mask;
    volatile u32 task_create_active_bit;
    volatile u32 free_heap_after_create;
    volatile u32 mpu6050_id;
    volatile u32 icm20948_id;
    volatile u32 hardware_version_bits;
    volatile u32 show_heartbeat;
    volatile u32 show_step;
    volatile u32 oled_refresh_enter;
    volatile u32 oled_refresh_exit;
    volatile u32 pwm_task_heartbeat;
    volatile u32 pwm_irq_count;
    volatile u32 pwm_spurious_irq_count;
    volatile u32 show_stack_min_words;
    volatile u32 pwm_stack_min_words;
    volatile u32 uart1_timeout_count;
    volatile u32 assert_line;
    const char * volatile assert_file;
    const char * volatile failed_task_name;
    volatile u32 fault_type;
    volatile u32 cfsr;
    volatile u32 hfsr;
    volatile u32 dfsr;
    volatile u32 afsr;
    volatile u32 mmfar;
    volatile u32 bfar;
    volatile u32 icsr;
    volatile u32 shcsr;
    volatile u32 msp;
    volatile u32 psp;
    volatile u32 ipsr;
    volatile u32 control;
} SYSTEM_DIAG_t;

extern volatile SYSTEM_DIAG_t g_system_diag;

void SystemDiag_Init(void);
void SystemDiag_SetStage(u32 stage);
void SystemDiag_SetError(u32 error_flag);
void SystemDiag_Log(const char *text);
void SystemDiag_Halt(u32 error_flag);
void SystemDiag_AssertFailed(const char *file, u32 line);
void SystemDiag_CaptureFault(u32 fault_type);

//�����˱���������ͷ�ļ�
#include "robot_init.h"

//Main logic code related header files
//���߼����ͷ�ļ�
#include "balance_task.h"
#include "imu_task.h"
#include "show_task.h"
#include "led_task.h"
#include "ps2_task.h"
#include "data_task.h"
#include "uartx_callback.h"
#include "apf_task.h"

//Check the multiple vehicle model definitions and ensure that only one model is allowed to exist at a time.
//�Զ��س��Ͷ�����,�������������ͬʱ����.
#if defined AKM_CAR + defined DIFF_CAR + defined MEC_CAR + defined _4WD_CAR + defined OMNI_CAR > 1
	#error "ERROR: multiple vehicle model definitions."
#endif

//����ӿ�
void systemInit(void);

//����������
extern TaskHandle_t g_reportErrTaskHandle;

/***Macros define***/ /***�궨��***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//����(1000/100hz=10)������������С�������˶�

//TODO:�ָ���1000
#define CONTROL_DELAY		1000

//RTOS����Ƶ��
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000


#endif 
