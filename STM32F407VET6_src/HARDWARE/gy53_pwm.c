#include "system.h"

GY53_PWM_SENSOR_t gy53_pwm_sensor[GY53_PWM_SENSOR_COUNT];
volatile u8 gy53_pwm_capture_enabled = 0;

static volatile u32 gy53_pwm_rise_cycle[GY53_PWM_SENSOR_COUNT];
static volatile u8 gy53_pwm_wait_fall[GY53_PWM_SENSOR_COUNT];

static u32 GY53_PWM_CyclesPerUs(void)
{
	u32 cycles_per_us = SystemCoreClock / 1000000U;
	if(cycles_per_us == 0) cycles_per_us = 168U;
	return cycles_per_us;
}

static void GY53_PWM_DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void GY53_PWM_SetRangerBoardValue(u8 sensor_id,u16 distance_mm)
{
	float distance_m = (float)distance_mm / 1000.0f;

	switch(sensor_id)
	{
		case GY53_PWM_FRONT: s21c_board.rangerA = distance_m; break;
		case GY53_PWM_LEFT:  s21c_board.rangerB = distance_m; break;
		case GY53_PWM_RIGHT: s21c_board.rangerC = distance_m; break;
		default: break;
	}
}

static void GY53_PWM_ConfigExtiLine(uint8_t pin_source,uint32_t exti_line)
{
	SYSCFG_EXTILineConfig(GY53_PWM_EXTI_PORT_SOURCE,pin_source);

	/* Configure the edge detector now, but keep delivery masked until RTOS runs. */
	EXTI->IMR &= ~exti_line;
	EXTI->EMR &= ~exti_line;
	EXTI->RTSR |= exti_line;
	EXTI->FTSR |= exti_line;
	EXTI->PR = exti_line;
}

static void GY53_PWM_HandleEdge(u8 sensor_id,u8 level)
{
	u32 now_cycle = DWT->CYCCNT;
	u32 pulse_us;
	u32 distance_mm;

	if(sensor_id >= GY53_PWM_SENSOR_COUNT) return;
	gy53_pwm_sensor[sensor_id].edge_count++;

	if(level)
	{
		gy53_pwm_rise_cycle[sensor_id] = now_cycle;
		gy53_pwm_wait_fall[sensor_id] = 1;
	}
	else if(gy53_pwm_wait_fall[sensor_id])
	{
		gy53_pwm_wait_fall[sensor_id] = 0;
		pulse_us = (now_cycle - gy53_pwm_rise_cycle[sensor_id]) / GY53_PWM_CyclesPerUs();
		gy53_pwm_sensor[sensor_id].last_pulse_us = pulse_us;

		if(pulse_us >= GY53_PWM_MIN_HIGH_US && pulse_us <= GY53_PWM_MAX_HIGH_US)
		{
			distance_mm = pulse_us / 10U;
			if(distance_mm > 65535U) distance_mm = 65535U;

			gy53_pwm_sensor[sensor_id].high_us = pulse_us;
			gy53_pwm_sensor[sensor_id].distance_mm = (u16)distance_mm;
			gy53_pwm_sensor[sensor_id].online = 1;
			gy53_pwm_sensor[sensor_id].updated = 1;
			gy53_pwm_sensor[sensor_id].update_count++;
		}
		else
		{
			gy53_pwm_sensor[sensor_id].invalid_count++;
		}
	}
}

void GY53_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GY53_PWM_DWT_Init();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GY53_PWM_FRONT_PIN | GY53_PWM_LEFT_PIN | GY53_PWM_RIGHT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GY53_PWM_ConfigExtiLine(GY53_PWM_FRONT_PIN_SOURCE,GY53_PWM_FRONT_EXTI_LINE);
	GY53_PWM_ConfigExtiLine(GY53_PWM_LEFT_PIN_SOURCE,GY53_PWM_LEFT_EXTI_LINE);
	GY53_PWM_ConfigExtiLine(GY53_PWM_RIGHT_PIN_SOURCE,GY53_PWM_RIGHT_EXTI_LINE);

	NVIC_InitStructure.NVIC_IRQChannel = GY53_PWM_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = GY53_PWM_EXTI_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(GY53_PWM_EXTI_IRQn,GY53_PWM_EXTI_PRIO);
	NVIC_DisableIRQ(GY53_PWM_EXTI_IRQn);
	NVIC_ClearPendingIRQ(GY53_PWM_EXTI_IRQn);
	gy53_pwm_capture_enabled = 0;
}

void GY53_PWM_EnableCapture(void)
{
	u8 i;

	EXTI->IMR &= ~GY53_PWM_EXTI_MASK;
	for(i=0;i<GY53_PWM_SENSOR_COUNT;i++)
	{
		gy53_pwm_wait_fall[i] = 0;
	}

	EXTI->PR = GY53_PWM_EXTI_MASK;
	NVIC_ClearPendingIRQ(GY53_PWM_EXTI_IRQn);
	gy53_pwm_capture_enabled = 1;
	EXTI->IMR |= GY53_PWM_EXTI_MASK;
	NVIC_EnableIRQ(GY53_PWM_EXTI_IRQn);
	g_system_diag.runtime_flags |= SYSTEM_DIAG_RUNTIME_PWM_CAPTURE;
}

void GY53_PWM_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	u8 i;
	u8 any_online;
	u32 now_tick;
	UBaseType_t stack_words;

	(void)pvParameters;
	vTaskDelay(M2T(GY53_PWM_CAPTURE_DELAY_MS));
	GY53_PWM_EnableCapture();
	lastWakeTime = getSysTickCnt();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(GY53_PWM_TASK_RATE));

		g_system_diag.pwm_task_heartbeat++;
		stack_words = uxTaskGetStackHighWaterMark(NULL);
		if(stack_words < g_system_diag.pwm_stack_min_words)
		{
			g_system_diag.pwm_stack_min_words = stack_words;
		}

		any_online = 0;
		now_tick = xTaskGetTickCount();
		for(i=0;i<GY53_PWM_SENSOR_COUNT;i++)
		{
			if(gy53_pwm_sensor[i].updated)
			{
				taskENTER_CRITICAL();
				if(gy53_pwm_sensor[i].updated)
				{
					gy53_pwm_sensor[i].updated = 0;
					gy53_pwm_sensor[i].last_update_tick = now_tick;
				}
				taskEXIT_CRITICAL();
			}

			if(gy53_pwm_sensor[i].last_update_tick != 0 &&
			  (now_tick - gy53_pwm_sensor[i].last_update_tick) <= M2T(GY53_PWM_ONLINE_TIMEOUT_MS))
			{
				gy53_pwm_sensor[i].online = 1;
				GY53_PWM_SetRangerBoardValue(i,gy53_pwm_sensor[i].distance_mm);
				any_online = 1;
			}
			else
			{
				gy53_pwm_sensor[i].online = 0;
			}
		}

		SysVal.HardWare_Ranger = any_online ? 1 : 0;
	}
}

void GY53_PWM_EXTI_IRQHandler(void)
{
	u32 pending = EXTI->PR & GY53_PWM_EXTI_MASK;

	g_system_diag.pwm_irq_count++;
	if(!gy53_pwm_capture_enabled)
	{
		if(pending != 0U) EXTI->PR = pending;
		g_system_diag.pwm_spurious_irq_count++;
		return;
	}

	if(pending & GY53_PWM_FRONT_EXTI_LINE)
	{
		EXTI->PR = GY53_PWM_FRONT_EXTI_LINE;
		GY53_PWM_HandleEdge(GY53_PWM_FRONT,GPIO_ReadInputDataBit(GY53_PWM_FRONT_PORT,GY53_PWM_FRONT_PIN));
	}

	if(pending & GY53_PWM_LEFT_EXTI_LINE)
	{
		EXTI->PR = GY53_PWM_LEFT_EXTI_LINE;
		GY53_PWM_HandleEdge(GY53_PWM_LEFT,GPIO_ReadInputDataBit(GY53_PWM_LEFT_PORT,GY53_PWM_LEFT_PIN));
	}

	if(pending & GY53_PWM_RIGHT_EXTI_LINE)
	{
		EXTI->PR = GY53_PWM_RIGHT_EXTI_LINE;
		GY53_PWM_HandleEdge(GY53_PWM_RIGHT,GPIO_ReadInputDataBit(GY53_PWM_RIGHT_PORT,GY53_PWM_RIGHT_PIN));
	}
}

void EXTI9_5_IRQHandler(void)
{
	GY53_PWM_EXTI_IRQHandler();
}
