#include "driver_d50a.h"

/*
 * D50A 串口协议固定为 13 字节：
 * AA AA AA ADDR FUNC D0 D1 D2 D3 BB BB BB BB
 *
 * 协议本身没有校验和，所以这里只能依赖：
 * 1. 固定帧头
 * 2. 固定帧尾
 * 3. 固定长度
 * 4. 地址/功能码匹配
 */
#define D50A_FRAME_LEN       13
#define D50A_HEAD            0xAA
#define D50A_TAIL            0xBB

/* 单条命令等待回复的最长时间，单位 ms。 */
#define D50A_TIMEOUT_MS      80

/* 其他任务最多可以排队的写请求数量。 */
#define D50A_REQ_QUEUE_LEN   8

/* D50A 协议功能码定义。 */
#define D50A_FUNC_READ_STATUS       0xA0
#define D50A_FUNC_READ_OVERCURRENT  0xA1
#define D50A_FUNC_READ_OVERTIME     0xA2
#define D50A_FUNC_READ_TEMP         0xA3
#define D50A_FUNC_READ_ADDR         0xA4
#define D50A_FUNC_WRITE_OVERCURRENT 0xB0
#define D50A_FUNC_WRITE_OVERTIME    0xB1
#define D50A_FUNC_RESET_PARAM       0xB2
#define D50A_FUNC_WRITE_ADDR        0xB3
#define D50A_FUNC_SAVE              0xB4

TaskHandle_t d50a_TaskHandle = NULL;

/* 写请求队列：蓝牙/OLED/其他任务只投递请求，不直接操作串口2。 */
static QueueHandle_t d50a_req_queue = NULL;

/* 多驱动器状态表。默认添加 0x01，后续可通过 D50A_AddDevice() 增加。 */
static D50A_State_t d50a_devices[D50A_MAX_DEVICES];
static uint8_t d50a_device_count = 0;

/* 自动轮询时使用的设备索引，每次轮询一个设备，避免单周期占用太久。 */
static uint8_t d50a_poll_device_index = 0;

/*
 * 当前正在通信的设备状态指针。
 * D50A_RequestAndWait() 根据它更新 online、last_error、last_update_tick。
 */
static D50A_State_t *d50a_active_state = NULL;

/*
 * USART2 中断接收缓冲。
 * rx_buf 用于正在接收的一帧。
 * rx_frame 用于保存已经收完整并通过帧尾检查的一帧。
 */
static volatile uint8_t rx_buf[D50A_FRAME_LEN];
static volatile uint8_t rx_frame[D50A_FRAME_LEN];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;

static void D50A_ClearRxFrame(void);
static void D50A_BuildFrame(uint8_t *frame,uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3);
static uint8_t D50A_CheckFrame(const uint8_t *frame);
static uint8_t D50A_IsExceptionFrame(const uint8_t *frame);
static uint8_t D50A_WaitFrame(uint8_t *frame,uint16_t timeout_ms);
static uint8_t D50A_WaitFrameNoRTOS(uint8_t *frame,uint16_t timeout_ms);
static void D50A_SendFrame(const uint8_t *frame);
static int16_t D50A_ToInt16(uint8_t high,uint8_t low);
static uint16_t D50A_ToUInt16(uint8_t high,uint8_t low);
static void D50A_UpdateAlarmBits(D50A_State_t *state,uint8_t alarm);
static void D50A_UpdateCommStats(D50A_State_t *state,uint8_t result);
static uint8_t D50A_ReturnFrameError(D50A_State_t *state);
static uint8_t D50A_ReturnSuccess(D50A_State_t *state);
static uint8_t D50A_PostRequest(const D50A_Request_t *request);
static uint8_t D50A_ProcessRequest(const D50A_Request_t *request);
static void D50A_PollStep(uint8_t step);
static D50A_State_t* D50A_FindState(uint8_t addr);
static D50A_State_t* D50A_FindAnyState(uint8_t addr);
static D50A_State_t* D50A_GetOrAddState(uint8_t addr);

static uint8_t D50A_ReadStatus(D50A_State_t *state);
static uint8_t D50A_ReadOverCurrent(D50A_State_t *state);
static uint8_t D50A_ReadOverTime(D50A_State_t *state);
static uint8_t D50A_ReadTemperature(D50A_State_t *state);
static uint8_t D50A_ReadAddress(void);
static uint8_t D50A_WriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma);
static uint8_t D50A_WriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms);
static uint8_t D50A_ResetParam(uint8_t addr);
static uint8_t D50A_WriteAddress(uint8_t old_addr,uint8_t new_addr);
static uint8_t D50A_Save(uint8_t addr);
static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp);

void D50A_Init(void)
{
	/* 清空设备表，并默认启用地址 0x01。 */
	memset(d50a_devices,0,sizeof(d50a_devices));
	d50a_device_count = 0;
	d50a_poll_device_index = 0;
	d50a_active_state = NULL;
	D50A_AddDevice(D50A_DEFAULT_ADDR);
	D50A_ClearRxFrame();

}

void D50A_Task(void *pvParameters)
{
	D50A_Request_t request;
	uint8_t poll_step = 0;
	u32 lastWakeTime = getSysTickCnt();

	/* 运行期任务启动后再创建请求队列，D50A_Init() 本身不依赖队列。 */
	if(d50a_req_queue == NULL)
	{
		d50a_req_queue = xQueueCreate(D50A_REQ_QUEUE_LEN,sizeof(D50A_Request_t));
	}

	while(1)
	{
		/* 任务 10Hz 运行，写请求优先，空闲时周期轮询状态。 */
		vTaskDelayUntil(&lastWakeTime,F2T(D50A_TASK_RATE));

		if(d50a_req_queue == NULL)
		{
			continue;
		}

		/*
		 * 优先处理外部投递的写请求。
		 * 一次任务周期只处理一条请求，保证通信节奏可控。
		 */
		if(xQueueReceive(d50a_req_queue,&request,0) == pdTRUE)
		{
			D50A_State_t *state = D50A_GetOrAddState(request.addr == D50A_BROADCAST_ADDR ? D50A_DEFAULT_ADDR : request.addr);
			if(state != NULL)
			{
				state->last_request = request.type;
				state->last_request_error = D50A_ProcessRequest(&request);
			}
		}
		else
		{
			/*
			 * 没有写请求时，执行自动轮询。
			 * poll_step 决定本次读哪类数据，D50A_PollStep() 决定本次访问哪个设备。
			 */
			D50A_PollStep(poll_step);
			poll_step++;
			if(poll_step >= 10) poll_step = 0;
		}
	}
}

void D50A_RxByte(uint8_t data)
{
	/*
	 * 收帧状态机：
	 * 1. 前3个字节必须连续收到 0xAA。
	 * 2. 收满13字节后检查第9~12字节是否为 0xBB。
	 * 3. 通过检查后复制到 rx_frame，并置位 rx_frame_ready。
	 *
	 * 这里运行在 USART2 中断中，只做极少量工作，不做业务解析。
	 */
	if(rx_index < 3)
	{
		if(data == D50A_HEAD)
		{
			rx_buf[rx_index++] = data;
		}
		else
		{
			rx_index = 0;
		}
		return;
	}

	rx_buf[rx_index++] = data;

	if(rx_index >= D50A_FRAME_LEN)
	{
		uint8_t i;
		if(rx_buf[9] == D50A_TAIL && rx_buf[10] == D50A_TAIL &&
		   rx_buf[11] == D50A_TAIL && rx_buf[12] == D50A_TAIL)
		{
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				rx_frame[i] = rx_buf[i];
			}
			rx_frame_ready = 1;
		}
		rx_index = 0;
	}
}

uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save)
{
	D50A_Request_t request;

	/* 上层传入单位为 mA，内部发送前再换算为协议单位 10mA。 */
	request.type = D50A_REQ_WRITE_OVERCURRENT;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ma;
	request.m2_value = m2_ma;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save)
{
	D50A_Request_t request;

	/* 上层传入单位为 ms，协议中也是 ms。 */
	request.type = D50A_REQ_WRITE_OVERTIME;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ms;
	request.m2_value = m2_ms;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostResetParam(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_RESET_PARAM;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_WRITE_ADDR;
	request.addr = old_addr;
	request.new_addr = new_addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostSave(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_SAVE;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostReadAddress(void)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));

	/*
	 * 广播查询地址只适合总线上单个驱动器。
	 * 多驱动器串联时不要调用该接口，否则多个设备可能同时回复。
	 */
	request.type = D50A_REQ_READ_ADDR;
	request.addr = D50A_BROADCAST_ADDR;
	return D50A_PostRequest(&request);
}

uint8_t D50A_SyncWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint16_t m1_raw,m2_raw;
	uint8_t ret;

	if(m1_ma < 1000 || m1_ma > 16000 || m2_ma < 1000 || m2_ma > 16000)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	m1_raw = m1_ma/10;
	m2_raw = m2_ma/10;
	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_WRITE_OVERCURRENT,m1_raw>>8,m1_raw,m2_raw>>8,m2_raw,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERCURRENT ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_raw || D50A_ToUInt16(resp[7],resp[8]) != m2_raw)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->m1_overcurrent_ma = m1_raw*10;
	d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	D50A_UpdateCommStats(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

uint8_t D50A_SyncWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	if(m1_ms < 10 || m1_ms > 10000 || m2_ms < 10 || m2_ms > 10000)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_WRITE_OVERTIME,m1_ms>>8,m1_ms,m2_ms>>8,m2_ms,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERTIME ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_ms || D50A_ToUInt16(resp[7],resp[8]) != m2_ms)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->m1_overtime_ms = m1_ms;
	d50a_active_state->m2_overtime_ms = m2_ms;
	D50A_UpdateCommStats(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

uint8_t D50A_SyncResetParam(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_RESET_PARAM,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_RESET_PARAM ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_SyncWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	D50A_State_t *new_state;

	if(new_addr == D50A_BROADCAST_ADDR || new_addr == 0)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(old_addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	new_state = D50A_FindState(new_addr);
	if(new_state != NULL && new_state != d50a_active_state)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(old_addr,D50A_FUNC_WRITE_ADDR,new_addr,0xFF,0xFF,0xFF,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != old_addr || resp[4] != D50A_FUNC_WRITE_ADDR || resp[5] != new_addr)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->addr = new_addr;
	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_SyncSave(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_AddDevice(uint8_t addr)
{
	D50A_State_t *state;

	/* 0 和广播地址不能作为普通设备地址加入轮询表。 */
	if(addr == 0 || addr == D50A_BROADCAST_ADDR)
		return D50A_ERR_PARAM;

	/* 如果设备槽位曾经存在但被禁用，则直接重新启用。 */
	state = D50A_FindAnyState(addr);
	if(state != NULL)
	{
		state->enable = 1;
		return D50A_OK;
	}

	if(d50a_device_count >= D50A_MAX_DEVICES)
		return D50A_ERR_QUEUE_FULL;

	/* 新建一个设备槽位，等待后续轮询更新状态。 */
	state = &d50a_devices[d50a_device_count++];
	memset(state,0,sizeof(D50A_State_t));
	state->enable = 1;
	state->addr = addr;
	state->last_error = D50A_ERR_NOT_INIT;
	return D50A_OK;
}

uint8_t D50A_RemoveDevice(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state == NULL)
		return D50A_ERR_PARAM;

	/* 不压缩数组，避免外部按 index 读取时状态突然移动。 */
	state->enable = 0;
	state->online = 0;
	return D50A_OK;
}

uint8_t D50A_GetDeviceCount(void)
{
	return d50a_device_count;
}

const D50A_State_t* D50A_GetStateByAddr(uint8_t addr)
{
	return D50A_FindState(addr);
}

const D50A_State_t* D50A_GetStateByIndex(uint8_t index)
{
	if(index >= d50a_device_count)
		return NULL;
	if(d50a_devices[index].enable == 0)
		return NULL;

	/* 返回只读指针，调用者不要强制转换后修改内部状态。 */
	return &d50a_devices[index];
}


static uint8_t D50A_PostRequest(const D50A_Request_t *request)
{
	if(d50a_req_queue == NULL)
	{
		return D50A_ERR_NOT_INIT;
	}
	if(xQueueSend(d50a_req_queue,request,0) != pdTRUE)
	{
		return D50A_ERR_QUEUE_FULL;
	}

	/*
	 * 请求入队成功后，提前把该地址加入设备表。
	 * 这样即使还没执行完成，外部也可以通过 D50A_GetStateByAddr() 找到状态槽位。
	 */
	if(request->addr != D50A_BROADCAST_ADDR)
	{
		D50A_AddDevice(request->addr);
	}
	return D50A_OK;
}

static void D50A_ClearRxFrame(void)
{
	/* 接收状态由中断和任务共同访问，清理时短暂关闭中断。 */
	__disable_irq();
	rx_index = 0;
	rx_frame_ready = 0;
	memset((void*)rx_buf,0,sizeof(rx_buf));
	memset((void*)rx_frame,0,sizeof(rx_frame));
	__enable_irq();
}

static void D50A_BuildFrame(uint8_t *frame,uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3)
{
	frame[0] = D50A_HEAD;
	frame[1] = D50A_HEAD;
	frame[2] = D50A_HEAD;
	frame[3] = addr;
	frame[4] = func;
	frame[5] = d0;
	frame[6] = d1;
	frame[7] = d2;
	frame[8] = d3;
	frame[9] = D50A_TAIL;
	frame[10] = D50A_TAIL;
	frame[11] = D50A_TAIL;
	frame[12] = D50A_TAIL;
}

static uint8_t D50A_CheckFrame(const uint8_t *frame)
{
	/* D50A 无校验和，只检查固定帧头和固定帧尾。 */
	if(frame[0] != D50A_HEAD || frame[1] != D50A_HEAD || frame[2] != D50A_HEAD)
		return 0;
	if(frame[9] != D50A_TAIL || frame[10] != D50A_TAIL || frame[11] != D50A_TAIL || frame[12] != D50A_TAIL)
		return 0;
	return 1;
}

static uint8_t D50A_IsExceptionFrame(const uint8_t *frame)
{
	/* 未定义功能码时，驱动器固定返回 AA AA AA AA FF FF FF FF FF BB BB BB BB。 */
	return frame[3] == 0xAA && frame[4] == 0xFF && frame[5] == 0xFF &&
	       frame[6] == 0xFF && frame[7] == 0xFF && frame[8] == 0xFF;
}

static uint8_t D50A_WaitFrame(uint8_t *frame,uint16_t timeout_ms)
{
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
	uint8_t i;

	while((xTaskGetTickCount() - start) <= timeout)
	{
		if(rx_frame_ready)
		{
			/* rx_frame 由中断写入，复制时进入临界区，复制完清 ready 标志。 */
			__disable_irq();
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				frame[i] = rx_frame[i];
			}
			rx_frame_ready = 0;
			__enable_irq();

			if(D50A_CheckFrame(frame) == 0)
				return D50A_ERR_FRAME;
			if(D50A_IsExceptionFrame(frame))
				return D50A_ERR_EXCEPTION;
			return D50A_OK;
		}
		vTaskDelay(1);
	}
	return D50A_ERR_TIMEOUT;
}

static uint8_t D50A_WaitFrameNoRTOS(uint8_t *frame,uint16_t timeout_ms)
{
	uint16_t wait_ms = 0;
	uint8_t i;

	/*
	 * 调度器启动前使用的等待函数。
	 * 这里不能调用 vTaskDelay()/xTaskGetTickCount()，只用 delay_ms(1) 做忙等超时。
	 * USART2 接收仍然依赖中断，收到完整帧后 D50A_RxByte() 会置位 rx_frame_ready。
	 */
	while(wait_ms <= timeout_ms)
	{
		if(rx_frame_ready)
		{
			__disable_irq();
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				frame[i] = rx_frame[i];
			}
			rx_frame_ready = 0;
			__enable_irq();

			if(D50A_CheckFrame(frame) == 0)
				return D50A_ERR_FRAME;
			if(D50A_IsExceptionFrame(frame))
				return D50A_ERR_EXCEPTION;
			return D50A_OK;
		}

		delay_ms(1);
		wait_ms++;
	}

	return D50A_ERR_TIMEOUT;
}

static void D50A_SendFrame(const uint8_t *frame)
{
	uint8_t i;

	/* 当前串口发送仍沿用工程原有阻塞式单字节发送风格。 */
	for(i=0;i<D50A_FRAME_LEN;i++)
	{
		uart2_send(frame[i]);
	}
}

static int16_t D50A_ToInt16(uint8_t high,uint8_t low)
{
	return (int16_t)(((uint16_t)high<<8)|low);
}

static uint16_t D50A_ToUInt16(uint8_t high,uint8_t low)
{
	return (uint16_t)(((uint16_t)high<<8)|low);
}

static void D50A_UpdateAlarmBits(D50A_State_t *state,uint8_t alarm)
{
	/* 协议规定：报警位为 1 表示正常，为 0 表示保护触发。 */
	state->alarm_bits.undervoltage_ok = (alarm & 0x08) ? 1 : 0;
	state->alarm_bits.overvoltage_ok = (alarm & 0x04) ? 1 : 0;
	state->alarm_bits.overtemp_ok = (alarm & 0x02) ? 1 : 0;
	state->alarm_bits.overcurrent_ok = (alarm & 0x01) ? 1 : 0;
}

static void D50A_UpdateCommStats(D50A_State_t *state,uint8_t result)
{
	if(state == NULL)
		return;

	/*
	 * 这三个计数只统计已经真正发到485总线上的通信结果。
	 * 参数错误、队列未初始化、队列满等本地错误不计入通信统计。
	 */
	if(result == D50A_OK)
	{
		state->comm_success_count++;
	}
	else if(result == D50A_ERR_TIMEOUT)
	{
		state->comm_timeout_count++;
	}
	else if(result == D50A_ERR_FRAME || result == D50A_ERR_EXCEPTION)
	{
		state->comm_frame_err_count++;
	}
}

static uint8_t D50A_ReturnFrameError(D50A_State_t *state)
{
	if(state != NULL)
	{
		state->online = 0;
		state->last_error = D50A_ERR_FRAME;
	}
	D50A_UpdateCommStats(state,D50A_ERR_FRAME);
	return D50A_ERR_FRAME;
}

static uint8_t D50A_ReturnSuccess(D50A_State_t *state)
{
	if(state != NULL)
	{
		state->online = 1;
		state->last_error = D50A_OK;
	}
	D50A_UpdateCommStats(state,D50A_OK);
	return D50A_OK;
}

static D50A_State_t* D50A_FindState(uint8_t addr)
{
	uint8_t i;

	/* 只查找启用中的设备。 */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].enable && d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

static D50A_State_t* D50A_GetOrAddState(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state != NULL)
		return state;

	/* 写请求访问新地址时，自动补进设备表。 */
	if(D50A_AddDevice(addr) != D50A_OK)
		return NULL;

	return D50A_FindState(addr);
}

static D50A_State_t* D50A_FindAnyState(uint8_t addr)
{
	uint8_t i;

	/* 查找所有槽位，包括已经 Remove/disable 的槽位。 */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret;

	/*
	 * 调度器启动前使用的同步收发函数。
	 * 该函数不走请求队列，不调用 vTaskDelay()，适合 systemInit() 阶段写驱动器参数。
	 * 注意：运行期如果 D50A_Task 已经启动，不建议再调用该函数，避免和任务同时访问485总线。
	 */
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	D50A_SendFrame(tx);
	ret = D50A_WaitFrameNoRTOS(resp,D50A_TIMEOUT_MS);

	if(d50a_active_state != NULL)
	{
		if(ret == D50A_OK)
		{
			d50a_active_state->online = 1;
			d50a_active_state->last_error = D50A_OK;
			d50a_active_state->last_update_tick = 0;
		}
		else
		{
			d50a_active_state->online = 0;
			d50a_active_state->last_error = ret;
			D50A_UpdateCommStats(d50a_active_state,ret);
		}
	}

	return ret;
}

static uint8_t D50A_RequestAndWait(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret;

	/*
	 * 发送命令前清空旧接收帧，避免上一帧残留被本次命令误用。
	 * 该函数只应由 D50A_Task 调用，保证总线上同一时刻只有一条命令。
	 */
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	D50A_SendFrame(tx);
	ret = D50A_WaitFrame(resp,D50A_TIMEOUT_MS);
	if(ret != D50A_OK)
	{
		if(d50a_active_state != NULL)
		{
			d50a_active_state->online = 0;
			d50a_active_state->last_error = ret;
			D50A_UpdateCommStats(d50a_active_state,ret);
		}
		return ret;
	}

	if(d50a_active_state != NULL)
	{
		d50a_active_state->online = 1;
		d50a_active_state->last_error = D50A_OK;
		d50a_active_state->last_update_tick = xTaskGetTickCount();
	}
	return D50A_OK;
}

static uint8_t D50A_ReadStatus(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_STATUS,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr) return D50A_ReturnFrameError(state);

	/*
	 * A0 返回帧特殊：第4字节不是功能码 A0，而是报警码。
	 * 后续4字节为 M1/M2 实时电流，int16，实际电流A = raw / 100。
	 */
	state->alarm = resp[4];
	D50A_UpdateAlarmBits(state,state->alarm);
	state->m1_current_raw = D50A_ToInt16(resp[5],resp[6]);
	state->m2_current_raw = D50A_ToInt16(resp[7],resp[8]);
	state->m1_current_a = state->m1_current_raw/100.0f;
	state->m2_current_a = state->m2_current_raw/100.0f;
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadOverCurrent(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERCURRENT,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERCURRENT) return D50A_ReturnFrameError(state);

	/* A1 返回的过流电流单位为 10mA，这里转换成 mA 存入缓存。 */
	state->m1_overcurrent_ma = D50A_ToUInt16(resp[5],resp[6])*10;
	state->m2_overcurrent_ma = D50A_ToUInt16(resp[7],resp[8])*10;
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadOverTime(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERTIME,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERTIME) return D50A_ReturnFrameError(state);

	/* A2 返回的过流时间单位为 ms。 */
	state->m1_overtime_ms = D50A_ToUInt16(resp[5],resp[6]);
	state->m2_overtime_ms = D50A_ToUInt16(resp[7],resp[8]);
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadTemperature(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_TEMP,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_TEMP) return D50A_ReturnFrameError(state);

	/* A3 温度格式：00 M1_TEMP 00 M2_TEMP。 */
	state->m1_temp = resp[6];
	state->m2_temp = resp[8];
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadAddress(void)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(D50A_BROADCAST_ADDR,D50A_FUNC_READ_ADDR,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[4] != D50A_FUNC_READ_ADDR) return D50A_ReturnFrameError(d50a_active_state);

	/* 查询到地址后自动加入设备表。多设备总线中不建议使用广播查询地址。 */
	D50A_AddDevice(resp[3]);
	d50a_active_state = D50A_FindState(resp[3]);
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint16_t m1_raw,m2_raw;
	uint8_t ret;

	if(m1_ma < 1000 || m1_ma > 16000 || m2_ma < 1000 || m2_ma > 16000)
		return D50A_ERR_PARAM;

	/* B0 协议单位为 10mA，上层 API 使用 mA。 */
	m1_raw = m1_ma/10;
	m2_raw = m2_ma/10;
	ret = D50A_RequestAndWait(addr,D50A_FUNC_WRITE_OVERCURRENT,m1_raw>>8,m1_raw,m2_raw>>8,m2_raw,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERCURRENT ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_raw || D50A_ToUInt16(resp[7],resp[8]) != m2_raw)
		return D50A_ReturnFrameError(d50a_active_state);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overcurrent_ma = m1_raw*10;
		d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	if(m1_ms < 10 || m1_ms > 10000 || m2_ms < 10 || m2_ms > 10000)
		return D50A_ERR_PARAM;

	/* B1 协议单位为 ms。 */
	ret = D50A_RequestAndWait(addr,D50A_FUNC_WRITE_OVERTIME,m1_ms>>8,m1_ms,m2_ms>>8,m2_ms,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERTIME ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_ms || D50A_ToUInt16(resp[7],resp[8]) != m2_ms)
		return D50A_ReturnFrameError(d50a_active_state);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overtime_ms = m1_ms;
		d50a_active_state->m2_overtime_ms = m2_ms;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_ResetParam(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_RESET_PARAM,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_RESET_PARAM ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	/* B2 会恢复默认过流参数并自动保存，后续轮询会读回最新值。 */
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	D50A_State_t *new_state;

	if(new_addr == D50A_BROADCAST_ADDR || new_addr == 0)
		return D50A_ERR_PARAM;

	/* B3 修改地址后驱动器会自动保存，不需要再发送 B4。 */
	ret = D50A_RequestAndWait(old_addr,D50A_FUNC_WRITE_ADDR,new_addr,0xFF,0xFF,0xFF,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != old_addr || resp[4] != D50A_FUNC_WRITE_ADDR || resp[5] != new_addr)
		return D50A_ReturnFrameError(d50a_active_state);

	new_state = D50A_FindState(new_addr);
	if(new_state != NULL && new_state != d50a_active_state)
		return D50A_ERR_PARAM;

	if(d50a_active_state != NULL)
	{
		d50a_active_state->addr = new_addr;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_Save(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	/* B4 只用于保存 B0/B1 写入的参数。 */
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_ProcessRequest(const D50A_Request_t *request)
{
	uint8_t ret;

	/* 处理写请求前，先确定本次通信要更新哪个设备的状态缓存。 */
	d50a_active_state = D50A_GetOrAddState(request->addr == D50A_BROADCAST_ADDR ? D50A_DEFAULT_ADDR : request->addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	switch(request->type)
	{
		case D50A_REQ_WRITE_OVERCURRENT:
			ret = D50A_WriteOverCurrent(request->addr,request->m1_value,request->m2_value);
			/* save_after_write 为 1 时，写成功后自动保存参数。 */
			if(ret == D50A_OK && request->save_after_write) ret = D50A_Save(request->addr);
			return ret;

		case D50A_REQ_WRITE_OVERTIME:
			ret = D50A_WriteOverTime(request->addr,request->m1_value,request->m2_value);
			/* save_after_write 为 1 时，写成功后自动保存参数。 */
			if(ret == D50A_OK && request->save_after_write) ret = D50A_Save(request->addr);
			return ret;

		case D50A_REQ_RESET_PARAM:
			return D50A_ResetParam(request->addr);

		case D50A_REQ_WRITE_ADDR:
			return D50A_WriteAddress(request->addr,request->new_addr);

		case D50A_REQ_SAVE:
			return D50A_Save(request->addr);

		case D50A_REQ_READ_ADDR:
			return D50A_ReadAddress();

		default:
			return D50A_ERR_PARAM;
	}
}

static void D50A_PollStep(uint8_t step)
{
	uint8_t ret;
	D50A_State_t *state;

	if(d50a_device_count == 0)
		return;

	if(d50a_poll_device_index >= d50a_device_count)
		d50a_poll_device_index = 0;

	state = &d50a_devices[d50a_poll_device_index];
	d50a_poll_device_index++;

	if(state->enable == 0)
		return;

	d50a_active_state = state;

	/*
	 * 自动轮询策略：
	 * D50A_Task 以 10Hz 运行，poll_step 每 10 次循环一轮。
	 *
	 * 单设备时：
	 * - A0 报警/实时电流：5Hz
	 * - A3 温度：2Hz
	 * - A1 过流电流设置值：1Hz
	 * - A2 过流时间设置值：1Hz
	 *
	 * 多设备时，每次只访问一个设备，所以单个设备频率约等于上述频率 / 设备数。
	 */
	switch(step)
	{
		case 0:
		case 2:
		case 4:
		case 6:
		case 8:
			ret = D50A_ReadStatus(state);
			break;
		case 1:
		case 5:
			ret = D50A_ReadTemperature(state);
			break;
		case 3:
			ret = D50A_ReadOverCurrent(state);
			break;
		case 7:
			ret = D50A_ReadOverTime(state);
			break;
		default:
			ret = D50A_OK;
			break;
	}

	if(ret != D50A_OK)
	{
		state->last_error = ret;
	}
}
