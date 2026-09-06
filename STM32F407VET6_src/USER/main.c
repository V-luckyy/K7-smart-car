/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.01
�޸�ʱ�䣺2024-06-25

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.01
Update��2024-06-25

All rights reserved
***********************************************/
#include "system.h"

//Task priority    //�������ȼ�
#define START_TASK_PRIO	25  // Must stay above USBH_PROCESS_PRIO (osPriorityNormal = 24).

//Task stack size //�����ջ��С	
#define START_STK_SIZE 	512  

//Task handle     //������
TaskHandle_t StartTask_Handler;

TaskHandle_t g_reportErrTaskHandle = NULL;

//Task function   //������
void start_task(void *pvParameters);
void ReportErrTask(void* param);

static void ShowRtosInitStep(const char *title)
{
	OLED_ClearBuf();
	OLED_ShowString(0,0,(const u8 *)title);
	OLED_ShowString(0,20,(const u8 *)"STAGE:");
	OLED_ShowNumber(48,20,g_system_diag.stage,5,12);
	OLED_ShowString(0,40,(const u8 *)"WATCH DIAG");
	g_system_diag.oled_refresh_enter++;
	OLED_Refresh_Gram();
	g_system_diag.oled_refresh_exit++;
}

static BaseType_t CreateTaskChecked(TaskFunction_t task,
                                    const char *name,
                                    uint16_t stack_depth,
                                    void *argument,
                                    UBaseType_t priority,
                                    TaskHandle_t *handle,
                                    u32 task_bit)
{
	BaseType_t result;

	g_system_diag.task_create_active_bit = task_bit;
	g_system_diag.failed_task_name = name;
	result = xTaskCreate(task,name,stack_depth,argument,priority,handle);
	if(result == pdPASS)
	{
		g_system_diag.task_ok_mask |= task_bit;
		g_system_diag.task_create_active_bit = 0U;
		g_system_diag.failed_task_name = NULL;
	}
	else
	{
		g_system_diag.task_fail_mask |= task_bit;
		SystemDiag_SetError(SYSTEM_DIAG_ERROR_TASK_CREATE);
		if(task_bit == SYSTEM_DIAG_TASK_START) SystemDiag_SetError(SYSTEM_DIAG_ERROR_START_TASK);
		SystemDiag_Halt(SYSTEM_DIAG_ERROR_TASK_CREATE);
	}
	return result;
}

//Main function //������
int main(void)
{
	systemInit();

	if(CreateTaskChecked(start_task,
	                     "start_task",
	                     START_STK_SIZE,
	                     NULL,
	                     START_TASK_PRIO,
	                     &StartTask_Handler,
	                     SYSTEM_DIAG_TASK_START) != pdPASS)
	{
		SystemDiag_Halt(SYSTEM_DIAG_ERROR_START_TASK);
	}

	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_START_TASK_READY);
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_SCHEDULER_START);
	SystemDiag_Log("[BOOT] starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	/* The scheduler must never return during normal operation. */
	SystemDiag_Halt(SYSTEM_DIAG_ERROR_SCHEDULER_RETURN);
	return 0;
}
 
//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
	(void)pvParameters;
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_START_TASK_RUN);

	if(SysVal.HardWare_Ver >= V1_1)
	{
		SystemDiag_SetStage(SYSTEM_DIAG_STAGE_USB_START);
		ShowRtosInitStep("RTOS: USB INIT");
		SystemDiag_Log("[RTOS] USB host init start\r\n");
		g_system_diag.task_create_active_bit = SYSTEM_DIAG_TASK_USB;
		g_system_diag.failed_task_name = "USBH_Queue";
		MX_USB_HOST_Init();
		g_system_diag.task_create_active_bit = 0U;
		g_system_diag.failed_task_name = NULL;
		g_system_diag.task_ok_mask |= SYSTEM_DIAG_TASK_USB;
		SystemDiag_SetStage(SYSTEM_DIAG_STAGE_USB_READY);
		SystemDiag_Log("[RTOS] USB host init ready\r\n");
	}

	ShowRtosInitStep("RTOS: TASK INIT");
	taskENTER_CRITICAL();

	CreateTaskChecked(Balance_task,  "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL,                 SYSTEM_DIAG_TASK_BALANCE);
	CreateTaskChecked(show_task,     "show_task",    SHOW_STK_SIZE,    NULL, SHOW_TASK_PRIO,    &show_TaskHandle,    SYSTEM_DIAG_TASK_SHOW);
	CreateTaskChecked(led_task,      "led_task",     LED_STK_SIZE,     NULL, LED_TASK_PRIO,     NULL,                 SYSTEM_DIAG_TASK_LED);
	CreateTaskChecked(data_task,     "DATA_task",    DATA_STK_SIZE,    NULL, DATA_TASK_PRIO,    &data_TaskHandle,     SYSTEM_DIAG_TASK_DATA);
	CreateTaskChecked(GY53_PWM_task, "GY53_PWM_task",GY53_PWM_STK_SIZE,NULL, GY53_PWM_TASK_PRIO,NULL,                 SYSTEM_DIAG_TASK_GY53);
	CreateTaskChecked(APF_task,      "APF_task",     APF_STK_SIZE,       NULL, APF_TASK_PRIO,    NULL,                 SYSTEM_DIAG_TASK_APF);

	if(SysVal.HardWare_Ver == V1_2)
	{
		CreateTaskChecked(D50A_Task, "D50A_task", D50A_STK_SIZE, NULL, D50A_TASK_PRIO, &d50a_TaskHandle, SYSTEM_DIAG_TASK_D50A);
	}

	if(SysVal.HardWare_Ver == V1_0)
	{
		CreateTaskChecked(MPU6050_task, "IMU_task",   IMU_STK_SIZE, NULL, IMU_TASK_PRIO, NULL, SYSTEM_DIAG_TASK_IMU);
		CreateTaskChecked(pstwo_task,   "PSTWO_task", PS2_STK_SIZE, NULL, PS2_TASK_PRIO, NULL, SYSTEM_DIAG_TASK_PS2);
	}
	else if(SysVal.HardWare_Ver >= V1_1)
	{
		CreateTaskChecked(ICM20948_task, "IMU_task", IMU_STK_SIZE, NULL, IMU_TASK_PRIO, NULL, SYSTEM_DIAG_TASK_IMU);
	}

	CreateTaskChecked(ReportErrTask, "ReportErrTask", 128*4, NULL, osPriorityNormal, &g_reportErrTaskHandle, SYSTEM_DIAG_TASK_REPORT);

	g_system_diag.free_heap_after_create = xPortGetFreeHeapSize();
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_TASKS_CREATED);
	taskEXIT_CRITICAL();

	SystemDiag_Log("[RTOS] task creation complete\r\n");
	vTaskDelete(NULL);
}
