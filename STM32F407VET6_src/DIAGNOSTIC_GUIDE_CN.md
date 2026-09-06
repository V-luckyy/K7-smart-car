# WHEELTEC C50X GY53 PWM 诊断增强指南

## 1. 烧录后正常启动顺序

上电后按顺序观察 OLED：

1. `BOOT: OLED OK`（约 0.8 秒）：主程序已完成板型识别并向 OLED 发送初始化和测试画面。
2. 新版硬件会显示 `RTOS: USB INIT`：调度器已运行，正在安全初始化 USB Host。
3. `RTOS: TASK INIT`：正在创建平衡、显示、数据、GY53 等业务任务。
4. `RTOS RUNNING`（约 0.8 秒）：FreeRTOS 和 `show_task` 已正常运行。
5. 随后进入原来的正常首页。

如果完全看不到第一屏，优先检查 OLED 供电、GND、排线、屏幕型号、硬件版本识别、I²C/IMU 初始化是否卡住。OLED 引脚随硬件版本变化，软件必须先识别 IMU 和板本版本才能选择正确引脚，因此发生在这一步之前的故障仍会表现为黑屏，只能结合 USART1 日志和 Keil Watch 定位。OLED 接口没有 ACK，因此程序只能证明“命令已经发出”，无法仅靠软件证明屏幕已收到。

## 2. OLED 第 4 页：三路 GY53 PWM

通过原有翻页操作切到第 4 页：

- `F/L/R`：前、左、右三个传感器。
- 第一行数据：距离，单位 `mm`；右侧 `ON/OFF` 表示最近是否持续收到有效 PWM。
- `P`：最近一次高电平脉宽，单位 `us`。
- `N`：有效距离更新次数（只显示末 4 位）。

判断方法：

- `P=0`、`N=0`：没有捕获到完整 PWM，检查传感器供电、共地、PWM 输出模式和 PE5/PE7/PE8 接线。
- `P` 变化但 `N=0`：有边沿，但脉宽不在有效范围内，检查信号制式、电平、噪声或换算关系。
- `N` 持续增加且 `ON`：该路传感器读取正常。
- 三路 `N` 都分别增加：三个传感器正在同时采集。

## 3. OLED 第 5 页：系统诊断

- `S`：当前启动/运行阶段码。
- `E`：错误标志十进制值。
- `T`：任务创建失败位掩码，正常为 0。
- `H`：任务创建后的剩余堆字节数。
- `OI/OO`：OLED 刷新进入/返回次数。第 5 页是在刷新调用内部读取计数，所以屏上正常情况通常是 `OI=OO+1`；在 Keil Watch 中，任务不处于刷新调用时两者应相等。
- `SH`：显示任务心跳，正常时持续增加。
- `ST`：显示任务正在执行的步骤。第 5 页正常刷新时会看到 5；要定位现场卡点应在 Keil Watch 中观察。
- `PW`：GY53 捕获是否已启用，正常为 1。
- `IQ`：GY53 EXTI 中断次数，有 PWM 边沿时应持续增加。
- `AD`：ADC 等待超时次数，正常为 0。
- `UR`：UART 发送超时次数，正常为 0。

## 4. Keil Watch 建议变量

进入 Debug 后，在 Watch 1 添加：

```text
g_system_diag
gy53_pwm_sensor
gy53_pwm_capture_enabled
SysVal.HardWare_Ver
ADC1_TimeoutCount
USART_TxTimeoutCount
```

重点展开：

- `g_system_diag.stage`：最后到达的启动步骤。
- `stage_trace[0..7]`：最近 8 个步骤。
- `error_flags`：错误位。
- `task_ok_mask/task_fail_mask/task_create_active_bit`：任务创建情况。
- `failed_task_name`：失败或栈溢出的任务名。
- `free_heap_after_create`：创建任务后剩余堆。
- `show_heartbeat/show_step`：显示任务是否活着、卡在哪一步。
- `oled_refresh_enter/oled_refresh_exit`：是否卡在 OLED 刷新。
- `pwm_task_heartbeat/pwm_irq_count/pwm_spurious_irq_count`：PWM 任务和中断状态。
- `show_stack_min_words/pwm_stack_min_words`：两个诊断重点任务历史最小剩余栈（单位为 32 位字）。
- `assert_file/assert_line`：FreeRTOS 断言位置。
- `fault_type/cfsr/hfsr/mmfar/bfar/msp/psp`：CPU Fault 信息。

三路传感器数组 `gy53_pwm_sensor[0..2]` 分别为前、左、右。每路重点看：

```text
distance_mm
last_pulse_us
edge_count
update_count
invalid_count
last_update_tick
online
```

## 5. 错误位和任务位

`error_flags`：

- bit0 (`0x001`)：IMU ID 无法识别。
- bit1 (`0x002`)：硬件版本引脚值非法。
- bit2 (`0x004`)：任务创建失败。
- bit3 (`0x008`)：启动任务创建失败。
- bit4 (`0x010`)：调度器异常返回。
- bit5 (`0x020`)：FreeRTOS 断言。
- bit6 (`0x040`)：堆内存申请失败。
- bit7 (`0x080`)：任务栈溢出。
- bit8 (`0x100`)：CPU Fault。
- bit9 (`0x200`)：USB Host 错误预留位。
- bit10 (`0x400`)：ADC 超时。
- bit11 (`0x800`)：UART 超时。

`fault_type`：1=HardFault，2=MemManage，3=BusFault，4=UsageFault，5=NMI。致命错误会立即关闭中断和电机输出并停机，OLED 不会继续刷新；`assert_file/assert_line`、`failed_task_name` 和 Fault 寄存器必须在 Keil Watch 中查看。

`task_ok_mask/task_fail_mask`：

- bit0：start
- bit1：Balance
- bit2：show
- bit3：LED
- bit4：DATA
- bit5：GY53 PWM
- bit6：D50A
- bit7：IMU
- bit8：PS2
- bit9：ReportErr
- bit10：USB Host

## 6. 启动阶段码

常用阶段：

- `0x0130`：正在识别 IMU。
- `0x0210/0x0211`：GY53 初始化开始/完成。
- `0x0220/0x0221`：OLED 初始化开始/完成。
- `0x02FF`：`systemInit()` 完成。
- `0x0310`：准备启动调度器。
- `0x0320`：`start_task` 已开始运行。
- `0x0140/0x0141`：USB Host 初始化开始/完成。
- `0x0330`：业务任务创建完成。
- `0x0400`：显示任务正常运行。
- `0xE000`：程序主动进入致命错误安全停机。
- `0xF000`：CPU Fault。

## 7. 串口启动日志

诊断日志固定走 USART1：

- TX：PA9
- RX：PA10
- 必须共地
- 115200、8 数据位、无校验、1 停止位

日志和原有串口发送现在都有超时保护，即使串口外设状态异常也不会永久卡死 OLED 任务。

## 8. 本次修改及用途

- 增加分阶段启动画面、串口日志和 `g_system_diag`：定位程序停在 OLED 前、调度器前、USB 还是任务创建阶段。
- 把 USB Host 从调度器启动前移到 `start_task`：避免 USB 中断在 FreeRTOS 尚未运行时调用 FromISR API。
- GY53 EXTI 在系统初始化时保持屏蔽，在 GY53 任务启动后再清 pending 并开启：避免调度器前的中断干扰。
- 明确写入 GY53 NVIC 优先级：避免 `NVIC_Init(...DISABLE)` 没有实际设置优先级而保留复位优先级 0。
- GY53 中断只做边沿时间戳和计数，不调用 FreeRTOS API：降低 ISR 死锁和断言风险。
- 增加第 4 页三传感器实时数据和第 5 页系统诊断。
- 所有 `xTaskCreate` 都检查返回值，并记录失败任务；开启 malloc 失败、栈溢出、FreeRTOS assert 和 CPU Fault 捕获。
- 致命错误时直接关闭 TIM8 主输出、四路比较输出和方向脚：防止调试停机后电机继续保持原 PWM。
- 初始化电机 PWM 的 SPL 配置结构体，并使用合法的 `TIM_CKD_DIV1`：消除未初始化字段导致的随机定时器配置。
- ADC 与 UART 忙等增加上限；ADC 失败后立即停止本轮 100 次采样；格式化发送改为有长度上限的 `vsnprintf`。

## 9. Keil 外部修改提醒

如果 Keil 弹出“文件已被外部修改”，必须选择 `Reload/重新加载`。不要选择保存旧编辑缓冲区，否则会覆盖本次修改。

