/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V3.5
修改时间：2021-01-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V3.5
Update：2021-01-29

All rights reserved
***********************************************/

#include "system.h"

//系统相关变量
SYS_VAL_t SysVal;

#define SYSTEM_DIAG_MAGIC 0x44494147UL
#define SYSTEM_DIAG_UART_TIMEOUT 100000UL

volatile SYSTEM_DIAG_t g_system_diag;

static void SystemDiag_Stop(void)
{
    __disable_irq();

    /* Stop wheel outputs without calling RTOS, delay, printf, or peripheral libraries. */
    if((RCC->APB2ENR & RCC_APB2ENR_TIM8EN) != 0U)
    {
        TIM8->BDTR &= ~((uint32_t)TIM_BDTR_MOE);
        TIM8->CCER = 0U;
        TIM8->CCR1 = 0U;
        TIM8->CCR2 = 0U;
        TIM8->CCR3 = 0U;
        TIM8->CCR4 = 0U;
        TIM8->CR1 &= ~((uint32_t)TIM_CR1_CEN);
    }

    /* Force PWM and direction pins to the same low state used at motor init. */
    if((RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN) != 0U)
    {
        GPIOC->BSRRH = (uint16_t)(GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_12);
        GPIOC->OTYPER &= ~((uint32_t)(GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9));
        GPIOC->PUPDR &= ~0x000FF000UL;
        GPIOC->MODER = (GPIOC->MODER & ~0x000FF000UL) | 0x00055000UL;
    }
    if((RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN) != 0U) GPIOA->BSRRH = GPIO_Pin_8;
    if((RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN) != 0U) GPIOB->BSRRH = (uint16_t)(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
    if((RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN) != 0U) GPIOD->BSRRH = (uint16_t)(GPIO_Pin_10 | GPIO_Pin_12);
    if((RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN) != 0U) GPIOE->BSRRH = GPIO_Pin_15;

    __DSB();
    __ISB();

    if((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }

    while(1)
    {
        __NOP();
    }
}

void SystemDiag_Init(void)
{
    u32 reset_csr = RCC->CSR;

    memset((void *)&g_system_diag,0,sizeof(g_system_diag));
    g_system_diag.magic = SYSTEM_DIAG_MAGIC;
    g_system_diag.reset_csr = reset_csr;
    g_system_diag.hardware_version_bits = 0xFFFFFFFFUL;
    g_system_diag.icm20948_id = 0xFFFFFFFFUL;
    g_system_diag.show_stack_min_words = 0xFFFFFFFFUL;
    g_system_diag.pwm_stack_min_words = 0xFFFFFFFFUL;

    RCC_ClearFlag();
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_RESET);
}

void SystemDiag_SetStage(u32 stage)
{
    u32 index;

    g_system_diag.previous_stage = g_system_diag.stage;
    g_system_diag.stage = stage;
    index = g_system_diag.trace_index % SYSTEM_DIAG_TRACE_DEPTH;
    g_system_diag.stage_trace[index] = stage;
    g_system_diag.trace_index++;
    __DSB();
}

void SystemDiag_SetError(u32 error_flag)
{
    g_system_diag.error_flags |= error_flag;
    __DSB();
}

void SystemDiag_Log(const char *text)
{
    u32 timeout;

    if(text == NULL) return;
    if((RCC->APB2ENR & RCC_APB2ENR_USART1EN) == 0U) return;
    if((USART1->CR1 & USART_CR1_UE) == 0U) return;

    while(*text != '\0')
    {
        timeout = SYSTEM_DIAG_UART_TIMEOUT;
        while((USART1->SR & USART_SR_TXE) == 0U)
        {
            if(--timeout == 0U)
            {
                g_system_diag.uart1_timeout_count++;
                g_system_diag.error_flags |= SYSTEM_DIAG_ERROR_UART_TIMEOUT;
                return;
            }
        }
        USART1->DR = (u8)(*text++);
    }
}

void SystemDiag_Halt(u32 error_flag)
{
    SystemDiag_SetError(error_flag);
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_FATAL);
    SystemDiag_Stop();
}

void SystemDiag_AssertFailed(const char *file,u32 line)
{
    g_system_diag.assert_file = file;
    g_system_diag.assert_line = line;
    SystemDiag_Halt(SYSTEM_DIAG_ERROR_ASSERT);
}

void SystemDiag_CaptureFault(u32 fault_type)
{
    g_system_diag.fault_type = fault_type;
    g_system_diag.cfsr = SCB->CFSR;
    g_system_diag.hfsr = SCB->HFSR;
    g_system_diag.dfsr = SCB->DFSR;
    g_system_diag.afsr = SCB->AFSR;
    g_system_diag.mmfar = SCB->MMFAR;
    g_system_diag.bfar = SCB->BFAR;
    g_system_diag.icsr = SCB->ICSR;
    g_system_diag.shcsr = SCB->SHCSR;
    g_system_diag.msp = __get_MSP();
    g_system_diag.psp = __get_PSP();
    g_system_diag.ipsr = __get_IPSR();
    g_system_diag.control = __get_CONTROL();
    SystemDiag_SetError(SYSTEM_DIAG_ERROR_CPU_FAULT);
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_CPU_FAULT);
    SystemDiag_Stop();
}

void vApplicationMallocFailedHook(void)
{
    if(g_system_diag.task_create_active_bit != 0U)
    {
        g_system_diag.task_fail_mask |= g_system_diag.task_create_active_bit;
        SystemDiag_SetError(SYSTEM_DIAG_ERROR_TASK_CREATE);
        if(g_system_diag.task_create_active_bit == SYSTEM_DIAG_TASK_START)
        {
            SystemDiag_SetError(SYSTEM_DIAG_ERROR_START_TASK);
        }
    }
    SystemDiag_Halt(SYSTEM_DIAG_ERROR_MALLOC);
}

void vApplicationStackOverflowHook(TaskHandle_t task,char *task_name)
{
    (void)task;
    g_system_diag.failed_task_name = task_name;
    SystemDiag_Halt(SYSTEM_DIAG_ERROR_STACK_OVERFLOW);
}

void systemInit(void)
{
    SystemDiag_Init();
	//================= General Hardware Initialization Section =================//
	//================= 通用硬件初始化部分 =================//
    //Interrupt priority group setting
    //中断优先级分组设置
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //Delay function initialization
    //延时函数初始化
    delay_init(168);
	
	//Prioritize initializing IIC and IMU C50C board distinguishes between new and old versions based on IMU model
	//优先初始化IIC与IMU.C50C板通过IMU型号区分新板与旧版
    //IIC initialization
    //IIC初始化
    I2C_GPIOInit();
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_I2C_READY);
	
	//系统相关软件参数初始化
	SYS_VAL_t_Init(&SysVal);
	
    //Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //串口1初始化，通信波特率115200，可用于与ROS端通信
    UART1_Init(115200);
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_UART1_READY);
    SystemDiag_Log("\r\n[BOOT] UART1 ready; watch g_system_diag\r\n");
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_IMU_DETECT);
	
	//如果IMU为MPU6050,则是旧版C50C
	g_system_diag.mpu6050_id = MPU6050_getDeviceID();
	if( MPU6050_DEFAULT_ADDRESS == g_system_diag.mpu6050_id )
	{
		SysVal.HardWare_Ver = V1_0;
		
		//旧版C50C硬件初始化
		V1_0_LED_Init();
		V1_0_CAN1_Mode_Init(1,3,3,6,0);
		V1_0_MiniBalance_PWM_Init(16799,0);
		
		//Initialize the hardware interface to the PS2 controller
		//初始化与PS2手柄连接的硬件接口
		PS2_Init();
		
		//PS2软件参数初始化
		PS2_Key_Param_Init();
		
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//MPU6050初始化，用于读取小车三轴角速度、三轴加速度信息
		MPU6050_initialize();
	}
	
	//如果IMU型号为ICM20948,则是新版C50C
	else if( REG_VAL_WIA == (g_system_diag.icm20948_id = ICM20948_getDeviceID()) )//读取ICM20948 id
	{
		//初始化版本接口
		Version_GPIO_Init();
		
		uint8_t ver=0;
		ver |= (VersionBit2)<<2;
		ver |= (VersionBit1)<<1;
		ver |= (VersionBit0)<<0;
		g_system_diag.hardware_version_bits = ver;
		
		switch( ver )
		{
			case 0:
				SysVal.HardWare_Ver = V1_1;
				break;
			case 1:
				SysVal.HardWare_Ver = V1_2;
				UART2_Init(115200); //v1.2版本后支持485接口
				D50A_Init();
				break;
			default:
				SystemDiag_Log("[ERR] invalid hardware version bits\r\n");
				SystemDiag_Halt(SYSTEM_DIAG_ERROR_HW_VERSION);
				break;
		}

	
		//Initialize the hardware interface connected to the LED lamp
		//初始化与LED灯连接的硬件接口
		RGB_LightStrip_Init();
		
		//Initialize the CAN communication interface
		//CAN通信接口初始化
		CAN1_Mode_Init(1,3,3,6,0);
		
		//Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
		//初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
		MiniBalance_PWM_Init(16799,0);  //高级定时器TIM8的频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
	
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//ICM20948初始化，用于读取小车三轴角速度、三轴加速度信息
		invMSInit();
		
		//USB PS2初始化
		/* USB host starts in start_task after FreeRTOS is running. *///创建usb手柄任务
	}
	else //无法识别的陀螺仪,复位系统
	{
		SystemDiag_Log("[ERR] IMU ID not recognized\r\n");
		SystemDiag_Halt(SYSTEM_DIAG_ERROR_IMU_ID);
	}
	
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_IMU_READY);
	SystemDiag_Log("[BOOT] IMU and board version ready\r\n");

    //Initialize the hardware interface connected to the buzzer
    //初始化与蜂鸣器连接的硬件接口
    Buzzer_Init();
    
    //Initialize the hardware interface connected to the enable switch
    //初始化与使能开关连接的硬件接口
    EnableKey_Init();

    //Initialize the hardware interface connected to the user's key
    //初始化与用户按键连接的硬件接口
    KEY_Init();
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_BASIC_IO_READY);
	
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_GY53_START);
    GY53_PWM_Init(); //GPIO/EXTI are configured here; IRQ remains disabled until the RTOS task starts.
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_GY53_READY);
    SystemDiag_Log("[BOOT] GY53 configured; capture IRQ is OFF\r\n");
    //Initialize the hardware interface connected to the OLED display
    //初始化与OLED显示屏连接的硬件接口
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_OLED_START);
    OLED_Init();
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_OLED_READY);

    /* This screen proves that code reached OLED initialization before RTOS starts. */
    OLED_ClearBuf();
    OLED_ShowString(0,0,(const u8 *)"BOOT: OLED OK");
    OLED_ShowString(0,16,(const u8 *)"HW:");
    OLED_ShowNumber(24,16,(u32)SysVal.HardWare_Ver,1,12);
    OLED_ShowString(0,32,(const u8 *)"STAGE:");
    OLED_ShowNumber(48,32,g_system_diag.stage,5,12);
    OLED_ShowString(0,48,(const u8 *)"PWM IRQ: OFF");
    g_system_diag.oled_refresh_enter++;
    OLED_Refresh_Gram();
    g_system_diag.oled_refresh_exit++;
    SystemDiag_Log("[BOOT] OLED commands and test screen sent\r\n");
    if(SYSTEM_DIAG_BOOT_SCREEN_MS > 0U) delay_ms(SYSTEM_DIAG_BOOT_SCREEN_MS);

    //Serial port 4 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //串口4初始化，通信波特率9600，用于与蓝牙APP端通信
	#if VECT_TAB_OFFSET == 0x10000 //根据是否配置无线烧录来选择蓝牙的波特率
		UART4_Init(230400);
	#elif VECT_TAB_OFFSET == 0
		UART4_Init(9600);
	#endif
	
    //Serial port 3 is initialized and the baud rate is 115200.
    //Serial port 3 is the default port used to communicate with ROS terminal
    //串口3初始化，通信波特率115200，串口3为默认用于与ROS端通信的串口
    UART3_Init(115200);
    SystemDiag_SetStage(SYSTEM_DIAG_STAGE_UARTS_READY);

    //Initialize the model remote control interface
    //初始化航模遥控接口
    Remoter_Init();
	
    //Encoder A is initialized to read the real time speed of motor A
    //编码器A初始化，用于读取电机A的实时速度
    EncoderA_Init();
    //Encoder B is initialized to read the real time speed of motor B
    //编码器B初始化，用于读取电机B的实时速度
    EncoderB_Init();
	
    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
    ADC1_Init();
	
	//阿克曼车型使用ADC2,不使用编码器C、D;其他车型反之
	#if defined AKM_CAR
		ADC2_Init();
	#else
		//Encoder C is initialized to read the real time speed of motor C  
		//编码器C初始化，用于读取电机C的实时速度	
		EncoderC_Init();
		//Encoder D is initialized to read the real time speed of motor D
		//编码器D初始化，用于读取电机D的实时速度	
		EncoderD_Init();  
	#endif
	
	//================= 软件参数初始化部分 =================//
	
	//确定机器人型号,初始化机器人机械参数和PID参数.
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_HARDWARE_READY);
	SystemDiag_Log("[BOOT] peripheral initialization ready\r\n");
	Robot_Select(); 
	
	//机器人控制相关变量初始化,包含遥控速度基准、最大速度限制、速度平滑系数等内容.
	ROBOT_CONTROL_t_Init(&robot_control); 
	
	//4个PI控制器初始化
	PI_Controller_Init(&PI_MotorA,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorB,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorC,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorD,robot.V_KP,robot.V_KI);
	
	//自动回充设备软件参数初始化
	auto_recharge_reset();
	
	//OLED软件参数初始化
	OLED_Param_Init(&oled);
	
	//APP软件参数初始化
	APPKey_Param_Init(&appkey);
	
	//航模遥控软件参数初始化
	Remoter_Param_Init(&remoter);
	
	//舵机参数初始化
	Akm_ServoParam_Init(&Akm_Servo);
	
	//从Flash读出舵机数据,若无数据则使用默认初始化值
	FlashParam_Read();    
	
	//阿克曼车型对舵机初始化
	#if defined AKM_CAR
	
	//高配阿克曼车型舵机初始化
	Servo_Senior_Init(10000-1,168-1,Akm_Servo.Mid);
	robot.SERVO.Output = Akm_Servo.Mid;
	
	//顶配阿克曼车型舵机初始化
	if( robot.type >= 2 && robot.type!= 9 )
	{
		//等待DMA采集数据
		delay_ms(200);
		
		//读取一组滑轨数据,估测舵机的位置.再将估测的位置作为PWM值初始化,可避免舵机突然快速归位.
		short TmpPWM = get_ServoPWM( get_DMA_SlideRes() );
		
		//顶配阿克曼车型舵机初始化,加入偏差值,避免舵机快速复位
		Servo_Top_Init(10000-1,84-1, TmpPWM );
		
		//舵机PI控制器初始化.注：舵机的PID参数不开放修改.
		PI_Controller_Init(&PI_Servo,0,0);
		
		//设置舵机PI控制基准值,加入偏差值,避免刚进入PI控制时舵机抖动
		PI_Servo.Output =  TmpPWM;
		
		//舵机速度平滑值
		robot_control.smooth_Servo = TmpPWM;
		
		//设置低速舵机模式,让舵机缓慢归位
		robot_control.ServoLow_flag = 1;
	}
	

	#endif
	
	//所有软硬件设备初始化完毕,使用蜂鸣器提示进入rtos
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_SYSTEM_READY);
	SystemDiag_Log("[BOOT] systemInit complete\r\n");
	Buzzer_AddTask(1,100);//蜂鸣1次,时间1000ms
}


