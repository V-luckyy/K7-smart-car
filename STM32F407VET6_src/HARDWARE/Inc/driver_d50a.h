#ifndef __DRIVER_D50A_H
#define __DRIVER_D50A_H

#include "system.h"

/*
 * D50A 驱动器管理模块
 *
 * 本模块统一管理 USART2/RS485 上的 D50A 驱动器通信。
 * 其他模块不要直接往串口2发送 D50A 协议帧，避免多个任务同时抢占485总线。
 * 如果需要修改驱动器参数，应调用 D50A_PostXXX() 系列接口把请求投递给 D50A_Task。
 * 如果需要查看驱动器状态，应读取 D50A_State_t 缓存，而不是主动发读命令。
 *
 * 多驱动串联注意事项：
 * 1. 总线上每个驱动器都必须有唯一地址。
 * 2. 广播地址 0xFF 只适合总线上只有一个驱动器时使用。
 * 3. 多个驱动器同时接在485总线上时，不要广播查询地址，否则多个设备会同时回复，造成总线冲突。
 */

#define D50A_TASK_PRIO       2
#define D50A_STK_SIZE        256
#define D50A_TASK_RATE       10

#define D50A_DEFAULT_ADDR    0x01
#define D50A_BROADCAST_ADDR  0xFF
#define D50A_MAX_DEVICES     4

/* 驱动器通信结果码：投递接口返回这些值，状态缓存里也会记录最近一次错误。 */
typedef enum
{
	D50A_OK = 0,
	D50A_ERR_TIMEOUT,
	D50A_ERR_FRAME,
	D50A_ERR_FUNC,
	D50A_ERR_PARAM,
	D50A_ERR_EXCEPTION,
	D50A_ERR_QUEUE_FULL,
	D50A_ERR_NOT_INIT,
}D50A_Result_t;

/* D50A_Task 内部处理的请求类型。 */
typedef enum
{
	D50A_REQ_WRITE_OVERCURRENT = 0,
	D50A_REQ_WRITE_OVERTIME,
	D50A_REQ_RESET_PARAM,
	D50A_REQ_WRITE_ADDR,
	D50A_REQ_SAVE,
	D50A_REQ_READ_ADDR,
}D50A_RequestType_t;

/*
 * 投递到 D50A_Task 队列中的请求对象。
 *
 * m1_value/m2_value 使用上层更容易理解的实际单位：
 * - 写过流电流时：单位为 mA，例如 8000 表示 8000mA
 * - 写过流时间时：单位为 ms，例如 5000 表示 5000ms
 *
 * save_after_write 仅对 B0/B1 参数写入有效：
 * - 0：只写入 RAM，不立刻保存
 * - 1：写入成功后自动追加 B4 保存命令
 */
typedef struct
{
	D50A_RequestType_t type;
	uint8_t addr;
	uint8_t new_addr;
	uint16_t m1_value;
	uint16_t m2_value;
	uint8_t save_after_write;
}D50A_Request_t;

/*
 * 报警位解析结果。
 *
 * D50A 协议中报警位的含义比较容易写反：
 * - 位值为 1 表示正常
 * - 位值为 0 表示对应保护触发
 */
typedef struct
{
	uint8_t undervoltage_ok;
	uint8_t overvoltage_ok;
	uint8_t overtemp_ok;
	uint8_t overcurrent_ok;
}D50A_AlarmBits_t;

/*
 * 单个 D50A 地址对应的状态缓存。
 *
 * D50A_Task 会周期读取驱动器数据并更新该结构体。
 * 蓝牙 APP、OLED 菜单或其他模块需要显示状态时，直接读取该缓存即可。
 * 不建议这些模块绕过 D50A_Task 直接访问 USART2。
 */
typedef struct
{
	uint8_t enable;              // 是否启用该设备槽位，0=不轮询，1=参与轮询
	uint8_t addr;                // 驱动器地址
	uint8_t online;              // 最近一次通信是否成功，1=在线，0=离线或超时
	uint8_t last_error;          // 最近一次周期读取的错误码
	uint8_t last_request;        // 最近一次处理的写请求类型
	uint8_t last_request_error;  // 最近一次写请求的执行结果

	uint8_t alarm;               // 原始报警码
	D50A_AlarmBits_t alarm_bits; // 报警码按位解析后的结果
	float m1_current_a;          // M1 实时电流，单位 A
	float m2_current_a;          // M2 实时电流，单位 A
	int16_t m1_current_raw;      // M1 实时电流原始值，协议值=实际电流A*100
	int16_t m2_current_raw;      // M2 实时电流原始值，协议值=实际电流A*100

	uint8_t m1_temp;             // M1 温度，单位 摄氏度
	uint8_t m2_temp;             // M2 温度，单位 摄氏度

	uint16_t m1_overcurrent_ma;  // M1 过流电流设置值，单位 mA
	uint16_t m2_overcurrent_ma;  // M2 过流电流设置值，单位 mA
	uint16_t m1_overtime_ms;     // M1 过流时间设置值，单位 ms
	uint16_t m2_overtime_ms;     // M2 过流时间设置值，单位 ms

	uint32_t last_update_tick;   // 最近一次成功通信的 FreeRTOS tick

	uint32_t comm_success_count;    // 累计通信成功次数：收到合法回复帧并完成协议校验后递增
	uint32_t comm_timeout_count;    // 累计通信超时次数：发送命令后在超时时间内未收到完整有效回复时递增
	uint32_t comm_frame_err_count;  // 累计返回数据错误次数：帧格式错误、异常帧、地址/功能码/数据不匹配时递增
}D50A_State_t;


/*
================================================================================
D50A API 调用示例和注意事项
================================================================================

一、总原则
--------------------------------------------------------------------------------
1. 串口2/485 总线由本模块统一管理，其他模块不要直接发送 D50A 协议帧。
2. FreeRTOS 启动前如果要写驱动器参数，使用 D50A_SyncXXX() 同步接口。
3. FreeRTOS 正常运行后，推荐使用 D50A_PostXXX() 异步接口，把请求交给 D50A_Task 处理。
4. 读取电流、温度、报警、参数设置值时，不需要主动发读命令，直接读取 D50A_State_t 缓存。
5. 多驱动器串联时，每个驱动器必须有唯一地址，例如 0x01、0x02、0x03。
6. 广播地址 0xFF 只能在总线上只有一个驱动器时用于查询地址，多驱动器时不要广播查询。

二、硬件初始化阶段：同步写初始参数
--------------------------------------------------------------------------------
适用场景：
- FreeRTOS 调度器还没有启动。
- 需要在 systemInit() 阶段给驱动器写入过流电流、过流时间等初始值。
- 此时不要使用 D50A_PostXXX()，因为 D50A_Task 还没有运行。

前置条件：
- UART2_Init(115200) 已经完成。
- D50A_Init() 已经完成。
- USART2 接收中断已经打开，并且中断函数中会调用 D50A_RxByte()。

示例：给地址 0x01 的驱动器设置默认参数，并立即保存。

    UART2_Init(115200);
    D50A_Init();

    // 设置 M1/M2 过流电流为 8000mA，并自动保存。
    // 第4个参数 save=1 表示写成功后自动发送 B4 保存命令。
    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 1);

    // 设置 M1/M2 过流时间为 5000ms，并自动保存。
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 1);

示例：连续写多个参数，最后统一保存。

    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 0);
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 0);
    D50A_SyncSave(0x01);

注意事项：
- 同步接口会阻塞等待驱动器返回，单条命令默认最长等待 D50A_TIMEOUT_MS。
- 同步接口适合初始化阶段使用；系统运行后不建议再调用，避免和 D50A_Task 同时访问485总线。
- 修改参数前，建议确保电机停转。

三、FreeRTOS 运行期：异步写请求
--------------------------------------------------------------------------------
适用场景：
- 蓝牙 APP、OLED 菜单、按键菜单等运行期修改参数。
- 不希望调用者阻塞等待串口回复。
- 希望所有写操作都由 D50A_Task 排队处理，避免485总线冲突。

示例：蓝牙 APP 设置地址 0x02 的过流电流为 8000mA，并保存。

    uint8_t ret;
    ret = D50A_PostWriteOverCurrent(0x02, 8000, 8000, 1);
    if(ret != D50A_OK)
    {
        // 这里只表示请求没有成功放入队列，例如队列满。
        // 不代表驱动器执行失败。
    }

示例：运行期写过流时间。

    D50A_PostWriteOverTime(0x02, 5000, 5000, 1);

示例：连续写多个参数，最后统一保存。

    D50A_PostWriteOverCurrent(0x02, 8000, 8000, 0);
    D50A_PostWriteOverTime(0x02, 5000, 5000, 0);
    D50A_PostSave(0x02);

查看异步写请求是否真正执行成功：

    const D50A_State_t *drv = D50A_GetStateByAddr(0x02);
    if(drv != NULL)
    {
        if(drv->last_request_error == D50A_OK)
        {
            // 最近一次写请求执行成功。
        }
        else
        {
            // 最近一次写请求执行失败，错误码见 D50A_Result_t。
        }
    }

注意事项：
- D50A_PostXXX() 的返回值只表示“投递队列是否成功”。
- 驱动器实际执行结果要看对应设备状态中的 last_request_error。
- D50A_Task 会优先处理写请求，处理写请求的周期会暂停一次自动轮询。

四、读取驱动器状态缓存
--------------------------------------------------------------------------------
D50A_Task 会周期读取：
- 报警码和实时电流 A0
- 温度 A3
- 过流电流设置值 A1
- 过流时间设置值 A2

调用者不需要主动发读命令，直接读取缓存即可。

示例：读取地址 0x01 的实时电流、温度和报警状态。

    const D50A_State_t *drv = D50A_GetStateByAddr(0x01);
    if(drv != NULL && drv->online)
    {
        float m1_current = drv->m1_current_a;   // M1 实时电流，单位 A
        float m2_current = drv->m2_current_a;   // M2 实时电流，单位 A
        uint8_t m1_temp = drv->m1_temp;         // M1 温度，单位 摄氏度
        uint8_t m2_temp = drv->m2_temp;         // M2 温度，单位 摄氏度

        if(drv->alarm_bits.overcurrent_ok == 0)
        {
            // 过流保护触发。
        }
        if(drv->alarm_bits.overtemp_ok == 0)
        {
            // 过温保护触发。
        }
    }

报警位注意事项：
- D50A 协议中，报警位为 1 表示正常。
- D50A 协议中，报警位为 0 表示对应保护触发。
- 例如 overcurrent_ok == 0 表示过流保护触发，不是正常。

五、多驱动器串联使用
--------------------------------------------------------------------------------
适用场景：
- 一条485总线上挂多个 D50A 驱动器。
- 每个驱动器使用不同地址。

示例：注册 3 个驱动器地址。

    D50A_AddDevice(0x01);
    D50A_AddDevice(0x02);
    D50A_AddDevice(0x03);

示例：按地址读取某个驱动器状态。

    const D50A_State_t *drv2 = D50A_GetStateByAddr(0x02);

示例：遍历所有已登记设备，适合批量显示在线状态。

    uint8_t i;
    for(i = 0; i < D50A_GetDeviceCount(); i++)
    {
        const D50A_State_t *drv = D50A_GetStateByIndex(i);
        if(drv != NULL)
        {
            // drv->addr          驱动器地址
            // drv->online        在线状态
            // drv->m1_temp       M1 温度
            // drv->m1_current_a  M1 电流
        }
    }

ByAddr 和 ByIndex 的区别：
- D50A_GetStateByAddr(addr)：按驱动器地址查状态，业务代码更推荐用这个。
- D50A_GetStateByIndex(index)：按内部设备表下标查状态，适合遍历所有设备。

多驱动注意事项：
- 不要让多个驱动器使用同一个地址。
- 不要在多驱动总线上使用 D50A_PostReadAddress() 广播查询地址。
- 如果要改地址，建议总线上只保留目标驱动器，避免误操作或地址冲突。

六、修改驱动器地址
--------------------------------------------------------------------------------
示例：把地址 0x01 修改为 0x02。

    D50A_PostWriteAddress(0x01, 0x02);

或在初始化阶段同步修改：

    D50A_SyncWriteAddress(0x01, 0x02);

注意事项：
- B3 修改地址命令会自动保存，不需要再调用 Save。
- 修改地址前，建议总线上只连接一个目标驱动器。
- 新地址不能为 0x00，也不能为广播地址 0xFF。
- 修改地址后，设备状态表中的地址也会更新。

七、恢复默认过流参数
--------------------------------------------------------------------------------
运行期异步恢复：

    D50A_PostResetParam(0x01);

初始化阶段同步恢复：

    D50A_SyncResetParam(0x01);

注意事项：
- B2 会恢复默认过流电流和过流时间，并自动保存。
- 恢复后，D50A_Task 后续轮询会重新读回设置值。
================================================================================
*/extern TaskHandle_t d50a_TaskHandle;

/* D50A 初始化：在支持485接口的硬件版本上，应在 UART2_Init() 之后调用。 */
void D50A_Init(void);

/*
 * D50A 管理任务：
 * 1. 统一处理所有写请求，避免多任务抢占485总线。
 * 2. 空闲时按照固定频率轮询驱动器状态。
 */
void D50A_Task(void *pvParameters);

/* USART2 接收中断中调用：把收到的一个字节喂给 D50A 收帧状态机。 */
void D50A_RxByte(uint8_t data);

/*
 * 异步写请求接口。
 *
 * 返回值只代表“请求是否成功放入队列”，不代表驱动器已经执行成功。
 * 驱动器实际执行结果请查看对应设备状态中的 last_request_error。
 */
uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_PostResetParam(uint8_t addr);
uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_PostSave(uint8_t addr);
uint8_t D50A_PostReadAddress(void);

/*
 * 硬件初始化阶段使用的同步写接口。
 *
 * 这些接口不依赖 D50A_Task，也不走 FreeRTOS 队列，适合在调度器启动前设置驱动器参数。
 * 使用前必须先完成 UART2_Init() 和 D50A_Init()，并保证 USART2 接收中断能调用 D50A_RxByte()。
 * 运行期仍建议优先使用 D50A_PostXXX()，避免和 D50A_Task 同时访问485总线。
 */
uint8_t D50A_SyncWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_SyncWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_SyncResetParam(uint8_t addr);
uint8_t D50A_SyncWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_SyncSave(uint8_t addr);

/* 多驱动器设备表管理接口。 */
uint8_t D50A_AddDevice(uint8_t addr);
uint8_t D50A_RemoveDevice(uint8_t addr);
uint8_t D50A_GetDeviceCount(void);
const D50A_State_t* D50A_GetStateByAddr(uint8_t addr);    // 按驱动器地址查询状态，适合业务代码使用
const D50A_State_t* D50A_GetStateByIndex(uint8_t index);  // 按设备表下标查询状态，适合遍历所有已登记设备


#endif
