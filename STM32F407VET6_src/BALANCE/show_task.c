#include "show_task.h"
#include "bsp_gamepad.h"

OLED_t oled;

void OLED_ShowGamePadState(void)
{
	oled.page = 0;
}

//OLED结构体初始化
void OLED_Param_Init(OLED_t* p)
{
	p->page = 1;
	p->last_page = 1;
	p->refrsh = 0;
	p->MAX_PAGE = OLED_MAX_PAGE;
}

/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
TaskHandle_t show_TaskHandle = NULL;
void show_task(void *pvParameters)
{
	u32 lastWakeTime;
	uint8_t i;
	uint8_t valid_samples;
	u32 adc_timeout_before;
	float sample;
	float tmp;
	UBaseType_t stack_words;

	(void)pvParameters;
	g_system_diag.runtime_flags |= SYSTEM_DIAG_RUNTIME_SHOW_TASK;
	SystemDiag_SetStage(SYSTEM_DIAG_STAGE_RTOS_RUNNING);
	g_system_diag.show_step = 1;

	/* First RTOS-owned frame: proves scheduler and show_task are running. */
	OLED_ClearBuf();
	OLED_ShowString(0,0,(const u8 *)"RTOS RUNNING");
	OLED_ShowString(0,16,(const u8 *)"TASK FAIL:");
	OLED_ShowNumber(72,16,g_system_diag.task_fail_mask,4,12);
	OLED_ShowString(0,32,(const u8 *)"FREE HEAP:");
	OLED_ShowNumber(80,32,g_system_diag.free_heap_after_create,5,12);
	OLED_ShowString(0,48,(const u8 *)"PWM:");
	OLED_ShowNumber(32,48,gy53_pwm_capture_enabled,1,12);
	g_system_diag.oled_refresh_enter++;
	OLED_Refresh_Gram();
	g_system_diag.oled_refresh_exit++;
	vTaskDelay(M2T(SYSTEM_DIAG_BOOT_SCREEN_MS));
	lastWakeTime = getSysTickCnt();

	while(1)
	{
		g_system_diag.show_step = 2;
		vTaskDelayUntil(&lastWakeTime,F2T(SHOW_TASK_RATE));
		g_system_diag.show_heartbeat++;
		stack_words = uxTaskGetStackHighWaterMark(NULL);
		if(stack_words < g_system_diag.show_stack_min_words)
		{
			g_system_diag.show_stack_min_words = stack_words;
		}
		if((USART_TxTimeoutCount != 0U) || (g_system_diag.uart1_timeout_count != 0U))
		{
			SystemDiag_SetError(SYSTEM_DIAG_ERROR_UART_TIMEOUT);
		}

		g_system_diag.show_step = 3;
		Buzzer_task(SHOW_TASK_RATE);

		g_system_diag.show_step = 4;
		tmp = 0;
		valid_samples = 0U;
		adc_timeout_before = ADC1_TimeoutCount;
		for(i=0;i<100;i++)
		{
			sample = Get_battery_volt();
			if(ADC1_TimeoutCount != adc_timeout_before) break;
			tmp += sample;
			valid_samples++;
		}
		if(ADC1_TimeoutCount != adc_timeout_before)
		{
			SystemDiag_SetError(SYSTEM_DIAG_ERROR_ADC_TIMEOUT);
		}
		if(valid_samples > 0U)
		{
			robot.voltage = tmp/(float)valid_samples;
			robot.voltage = VolMean_Filter(robot.voltage);
		}

		if(robot.voltage < 20 && robot.LowPower == 0) robot.LowPower = 1;
		if(robot.voltage > 21.0f) robot.LowPower = 0;

		if(robot.LowPower && robot.voltage > 18.0f && robot.voltage < 20.0f)
		{
			static uint16_t buzzer_timer = 0;
			buzzer_timer++;
			if(buzzer_timer >= 10)
			{
				Buzzer_AddTask(1,50);
				buzzer_timer = 0;
			}
		}

		if(1 == robot.LowPower && 1 == SysVal.HardWare_charger && charger.RED_STATE >= 2)
		{
			if(SysVal.Time_count > CONTROL_DELAY) charger.AllowRecharge = 1;
		}

		/* Refresh OLED before APP output so a UART fault cannot hide the display. */
		g_system_diag.show_step = 5;
		g_system_diag.oled_refresh_enter++;
		OLED_ShowTask();
		g_system_diag.oled_refresh_exit++;

		g_system_diag.show_step = 6;
		if(Debug_Flag == 0) APP_ShowTask();
		g_system_diag.show_step = 0;
	}
}

/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
//电量
int voltage_to_percentage(float voltage) {
    const float V_MIN = 20.0f;  // 最小电压 (0%)
    const float V_MAX = 23.05f; // 最大电压 (100%)
    
    // 确保电压在有效范围内
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // 线性插值计算百分比并取整
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // 四舍五入
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

static void APP_ShowTask(void)
{
    static u8 flag_show;
    int Left_Figure,Right_Figure;

    //The battery voltage is processed as a percentage
    //对电池电压处理成百分比形式
	static int Voltage_Show=100;
	static uint32_t delayshowVol=0;
	
	delayshowVol++;
	if( delayshowVol >= 10*10 || delayshowVol==1 )
	{
		delayshowVol=1;
		Voltage_Show = voltage_to_percentage(robot.voltage);
	}

    //Wheel speed unit is converted to 0.01m/s for easy display in APP
    //车轮速度单位转换为0.01m/s，方便在APP显示
	#if defined AKM_CAR || defined DIFF_CAR //阿克曼、差速显示A、B轮
		Left_Figure=robot.MOTOR_A.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_B.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#elif defined MEC_CAR || defined _4WD_CAR //麦轮、四驱显示A、D轮
		Left_Figure=robot.MOTOR_A.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_D.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#else                                      //全向轮显示B、C轮
		Left_Figure=robot.MOTOR_B.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_C.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#endif

    //Used to alternately print APP data and display waveform
    //用于交替打印APP数据和显示波形
    flag_show=!flag_show;

    if(appkey.ParamSendflag==1)
    {
        //Send parameters to the APP, the APP is displayed in the debug screen
        //发送参数到APP，APP在调试界面显示
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
               (int)robot_control.rc_speed,
               (int)robot.V_KP,
               (int)robot.V_KI,
               (int)(robot_control.smooth_MotorStep*1000),
               (int)(robot_control.smooth_ServoStep),
               (int)(Akm_Servo.Max),
               (int)(Akm_Servo.Min),
               (int)(Akm_Servo.Mid),
			   robot_control.LineDiffParam);
				   

        appkey.ParamSendflag=0;
    }
    else if(flag_show==0)
    {
        //Send parameters to the APP and the APP will be displayed on the front page
        //发送参数到APP，APP在首页显示
        printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);

    }
    else
    {
        //Send parameters to the APP, the APP is displayed in the waveform interface
        //发送参数到APP，APP在波形界面显示
        printf("{B%d:%d:%d}$",(int)(imu.accel.x),(int)(imu.accel.y),(int)(imu.accel.z));
    }
	
//	if(Debug_Flag)
//	{
//		if(IP_Frame_Valid)//ip请求已关闭且ip以确认
//			printf("{#IP:%d.%d.%d.%d}$",(u8)Received_IP[0],(u8)Received_IP[1],(u8)Received_IP[2],(u8)Received_IP[3]);//上发IP
//		else printf("{#IP:unknow..}$");//ip未知
//		Debug_Flag=0;
//	}
}

/**************************************************************************
Functionality: Battery voltage sliding filter function.
Input Parameters: Collected battery voltage data.
Return Value: Filtered battery voltage data.
Author: WHEELTEC
函数功能：电池电压滑动滤波函数
入口参数：采集到的电池电压数据
返回  值：经过滤波后的电池电压数据
作    者：WHEELTEC
**************************************************************************/
#define VOL_COUNT 100
static float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};

    /*----------- 数组初始化 -----------*/
    static u8 once=1;
    if(once)
    {
        once=0;
        for(i=0; i<VOL_COUNT; i++)
            Speed_Buf[i]= robot.voltage ;
    }
    /*-------------------------------*/

    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
static void OLED_ShowTask(void)
{
	//检测OLED页面是否需要刷新
	if( oled.page!=oled.last_page ) 
	{
		OLED_Clear(),OLED_Refresh_Gram();
		oled.last_page = oled.page;
		return;
	}
	oled.last_page = oled.page;
	
	//首页显示小车的各个传感器信息
	if( 1 == oled.page ) 
	{
		//公共显示信息,首行车型,末行电池电压与控制方式
		uint16_t TypeNum = 4096/CAR_NUMBER;
		TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
		
		//第一行左半部分,显示电位器车型.同时带显示是否正在自动回充功能
		if( 0 == charger.AllowRecharge) OLED_ShowString(0,0,(const u8 *)"TYPE:");
		else                           OLED_ShowString(0,0,(const u8 *)"RCM :");
		
		//车型号,报错则显示X
		if( 0 == robot_check.errorflag) OLED_ShowNumber(40,0,TypeNum,2,12);
		else                           OLED_ShowString(38,0,(const u8 *)" X");
		
		//第一行右半部分,Z轴角速度
		OLED_ShowString(60,0,(const u8 *)"GZ");
		if( imu.gyro.z < 0)  OLED_ShowString(80,0,(const u8 *)"-"),OLED_ShowNumber(90,0,-imu.gyro.z,5,12);
		else                 OLED_ShowString(80,0,(const u8 *)"+"),OLED_ShowNumber(90,0, imu.gyro.z,5,12);
		//oled_showfloat(debug.u8_val,80,0,3,2);
		
		
		//最后一行左半部分,显示控制类型 和 显示小车是否允许被控制
			  if( Get_Control_Mode(_ROS_Control) )    OLED_ShowString(0,50,(const u8 *)"ROS  ");
		else if(  Get_Control_Mode(_PS2_Control) )   OLED_ShowString(0,50,(const u8 *)"PS2  ");
		else if(  Get_Control_Mode(_APP_Control) )   OLED_ShowString(0,50,(const u8 *)"APP  ");
		else if(  Get_Control_Mode(_RC_Control)  )   OLED_ShowString(0,50,(const u8 *)"R-C  ");
		else if(  Get_Control_Mode(_CAN_Control) )   OLED_ShowString(0,50,(const u8 *)"CAN  ");
		else if(  Get_Control_Mode(_USART_Control) ) OLED_ShowString(0,50,(const u8 *)"USART");
		
		//显示小车是否允许被控制
		if( 0 == robot_control.FlagStop ) OLED_ShowString(45,50,(const u8 *)" ON");
		else                              OLED_ShowString(45,50,(const u8 *)"OFF");
			
		//右半部分显示电池电压
		oled_showfloat(robot.voltage,75,50,2,2);
		OLED_ShowString(75,50,(const u8 *)" ");
		OLED_ShowString(120,50,(const u8 *)"V");
		
		//第2、3、4、5行非公共区域,根据车型不同显示的特定信息
		#if defined AKM_CAR
		oled_akm_show();
		#elif defined DIFF_CAR
			oled_diff_show();
		#elif defined MEC_CAR || defined _4WD_CAR || defined OMNI_CAR
			oled_mec_4wd_omni_show();
		#endif
	}
	
	//第二页显示自动回充调试信息
	else if( 2 == oled.page ) 
	{
		//自动回充套件Debug信息
		OLED_ShowString(07,00,(const u8 *)"LA  LB  RB  RA");
		OLED_ShowNumber(0+9,10,charger.L_A,1,12);
		OLED_ShowNumber(30+9,10,charger.L_B,1,12);
		OLED_ShowNumber(60+9,10,charger.R_B,1,12);
		OLED_ShowNumber(90+9,10,charger.R_A,1,12);
		OLED_ShowString(0,30,(const u8 *)"cur:"); 
		OLED_ShowString(75,30,(const u8 *)"A"); 
		oled_showfloat(charger.ChargingCurrent/1000.0f,30,30,2,2);
	}
	
	//第三页显示小车的版本
	else if( 3 == oled.page )
	{
		//第1行 显示是哪种车
		OLED_ShowString(0,0,(const u8 *)"CarMode:");
		#if defined AKM_CAR
			OLED_ShowString(66,0,"AKM");
		#elif defined DIFF_CAR
			OLED_ShowString(66,0,(const u8 *)"DIFF");
		#elif defined MEC_CAR
			OLED_ShowString(66,0,"MEC");
		#elif defined _4WD_CAR
			OLED_ShowString(66,0,"4WD");
		#elif defined OMNI_CAR
			OLED_ShowString(66,0,"OMNI");
		#endif
		
		//第二行显示车型代号
		OLED_ShowString(0,12,(const u8 *)"CarType:");
		OLED_ShowNumber(66,12,robot.type,2,12);
		
		//第三行显示硬件版本
		OLED_ShowString(0,24,(const u8 *)"HW_Ver:");
		OLED_ShowString(66,24,getHW_Ver(SysVal.HardWare_Ver));
		
		//第四行显示软件版本
		OLED_ShowString(0,36,(const u8 *)"SW_Ver:");
		OLED_ShowString(66,36,getSW_Ver(SysVal.SoftWare_Ver));
		
		//IP显示
		OLED_ShowString(0,48,(const u8 *)"IP:");
		OLED_ShowNumber(24,48,Received_IP[0],3,12);
		OLED_ShowNumber(48,48,Received_IP[1],3,12);
		OLED_ShowNumber(72,48,Received_IP[2],3,12);
		OLED_ShowNumber(96,48,Received_IP[3],3,12);
	}
	
	else if( 4 == oled.page )
	{
		/* F/L/R: distance and online state. P/N: pulse width and valid update count. */
		OLED_ShowString(0,0,(const u8 *)"F:");
		OLED_ShowNumber(16,0,gy53_pwm_sensor[GY53_PWM_FRONT].distance_mm,5,12);
		OLED_ShowString(48,0,(const u8 *)"mm");
		OLED_ShowString(72,0,gy53_pwm_sensor[GY53_PWM_FRONT].online ? (const u8 *)"ON " : (const u8 *)"OFF");
		OLED_ShowString(0,10,(const u8 *)"P:");
		OLED_ShowNumber(16,10,gy53_pwm_sensor[GY53_PWM_FRONT].last_pulse_us,5,12);
		OLED_ShowString(48,10,(const u8 *)"us");
		OLED_ShowString(72,10,(const u8 *)"N:");
		OLED_ShowNumber(88,10,gy53_pwm_sensor[GY53_PWM_FRONT].update_count%10000U,4,12);

		OLED_ShowString(0,20,(const u8 *)"L:");
		OLED_ShowNumber(16,20,gy53_pwm_sensor[GY53_PWM_LEFT].distance_mm,5,12);
		OLED_ShowString(48,20,(const u8 *)"mm");
		OLED_ShowString(72,20,gy53_pwm_sensor[GY53_PWM_LEFT].online ? (const u8 *)"ON " : (const u8 *)"OFF");
		OLED_ShowString(0,30,(const u8 *)"P:");
		OLED_ShowNumber(16,30,gy53_pwm_sensor[GY53_PWM_LEFT].last_pulse_us,5,12);
		OLED_ShowString(48,30,(const u8 *)"us");
		OLED_ShowString(72,30,(const u8 *)"N:");
		OLED_ShowNumber(88,30,gy53_pwm_sensor[GY53_PWM_LEFT].update_count%10000U,4,12);

		OLED_ShowString(0,40,(const u8 *)"R:");
		OLED_ShowNumber(16,40,gy53_pwm_sensor[GY53_PWM_RIGHT].distance_mm,5,12);
		OLED_ShowString(48,40,(const u8 *)"mm");
		OLED_ShowString(72,40,gy53_pwm_sensor[GY53_PWM_RIGHT].online ? (const u8 *)"ON " : (const u8 *)"OFF");
		OLED_ShowString(0,50,(const u8 *)"P:");
		OLED_ShowNumber(16,50,gy53_pwm_sensor[GY53_PWM_RIGHT].last_pulse_us,5,12);
		OLED_ShowString(48,50,(const u8 *)"us");
		OLED_ShowString(72,50,(const u8 *)"N:");
		OLED_ShowNumber(88,50,gy53_pwm_sensor[GY53_PWM_RIGHT].update_count%10000U,4,12);
	}

	else if( 5 == oled.page )
	{
		/* Compact labels avoid overlap on the 128-pixel display. */
		OLED_ShowString(0,0,(const u8 *)"S:");
		OLED_ShowNumber(16,0,g_system_diag.stage,5,12);
		OLED_ShowString(56,0,(const u8 *)"E:");
		OLED_ShowNumber(72,0,g_system_diag.error_flags%10000U,4,12);

		OLED_ShowString(0,10,(const u8 *)"T:");
		OLED_ShowNumber(16,10,g_system_diag.task_fail_mask,4,12);
		OLED_ShowString(56,10,(const u8 *)"H:");
		OLED_ShowNumber(72,10,g_system_diag.free_heap_after_create,5,12);

		OLED_ShowString(0,20,(const u8 *)"OI:");
		OLED_ShowNumber(24,20,g_system_diag.oled_refresh_enter%10000U,4,12);
		OLED_ShowString(64,20,(const u8 *)"OO:");
		OLED_ShowNumber(88,20,g_system_diag.oled_refresh_exit%10000U,4,12);

		OLED_ShowString(0,30,(const u8 *)"SH:");
		OLED_ShowNumber(24,30,g_system_diag.show_heartbeat%10000U,4,12);
		OLED_ShowString(64,30,(const u8 *)"ST:");
		OLED_ShowNumber(88,30,g_system_diag.show_step,1,12);

		OLED_ShowString(0,40,(const u8 *)"PW:");
		OLED_ShowNumber(24,40,gy53_pwm_capture_enabled,1,12);
		OLED_ShowString(56,40,(const u8 *)"IQ:");
		OLED_ShowNumber(80,40,g_system_diag.pwm_irq_count%10000U,4,12);

		OLED_ShowString(0,50,(const u8 *)"AD:");
		OLED_ShowNumber(24,50,ADC1_TimeoutCount%1000U,3,12);
		OLED_ShowString(56,50,(const u8 *)"UR:");
		OLED_ShowNumber(80,50,(USART_TxTimeoutCount+g_system_diag.uart1_timeout_count)%1000U,3,12);
	}

	/* Page 0 is reserved for USB gamepad status. */
	else if( 0 == oled.page )
	{
		if( GamePadDebug.enmu_state == EnumWait ) //枚举等待中
		{
			OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);//插入usb提示
			OLED_ShowString(12,50,(const u8 *)"USB Init..");
			OLED_Refresh_Line();
			return;
		}
		else if( GamePadDebug.enmu_state == EnumNULL )
		{
			OLED_ClearBuf();
			oled.page = 1;
		}
		else if( GamePadDebug.enmu_state == EnumDone ) //枚举完成
		{
			//枚举完毕,显示枚举结果后,等待一段时间恢复正常显示
			static uint16_t showtime=0;
			if(++showtime >= SHOW_TASK_RATE*3 )
			{
				showtime = 0;
				oled.page = 1;
			}
			
			//清空缓冲区但不刷新,等待写入新的内容
			OLED_ClearBuf();
			
			//ps2初始化情况
			OLED_ShowString(0,0,(const u8 *)"USB Init OK.");
			OLED_ShowString(0,15,(const u8 *)"PS2 Info:");
			
			if( GamePadDebug.type == PS2_USB_Wired || GamePadDebug.type == PS2_USB_WiredV2 )
			{
				OLED_ShowString(0,30,(const u8 *)"Wired USBPS2");
			}
			else if( GamePadDebug.type == PS2_USB_Wiredless )
			{
				OLED_ShowString(0,30,(const u8 *)"2.4G USBPS2 ");
			}
			else if( GamePadDebug.type == Xbox360 )
			{
				OLED_ShowString(0,30,(const u8 *)"xbox 360    ");
			}
			else OLED_ShowString(0,30,(const u8 *)"UnKnown Dev ");
			
			//是否成功获取到ps2的数据
			OLED_ShowString(0,45,(const u8 *)"Data Ready:");
			if( 1 == GamePadDebug.ready ) //成功获取ps2数据
			{
				OLED_ShowString(90,45,(const u8 *)"Yes");
			}
			else //无数据
			{
				OLED_ShowString(90,45,(const u8 *)"No ");
			}
			
		}
	}
	
	//oled刷新
	OLED_Refresh_Gram();
}

extern int PengZhuang;
//不同车型对应的显示页面
#if defined AKM_CAR
static void oled_akm_show(void)
{
	//工程测试模式,用于机械安装使用
	if( robot.type==7 || robot.type==8 )
	{
		OLED_ShowString(00,10,"SLID:");
		if( robot.SERVO.Encoder < 0 )	OLED_ShowString(60,10,"-"),
								OLED_ShowNumber(80,10,-(int)robot.SERVO.Encoder,4,12);
		else                 	OLED_ShowString(60,10,"+"),
								OLED_ShowNumber(80,10, (int)robot.SERVO.Encoder,4,12); 
	}
	else
	{
		OLED_ShowString(00,10,"PZ");
		oled_showfloat(PengZhuang*1.0f,80,10,3,1);
	}
	
	 //The third line of the display displays the content//
	 //显示屏第3行显示内容//		
	 //Display the target speed and current speed of motor A
	 //显示电机A的目标速度和当前速度
	 OLED_ShowString(0,20,"L:");
	 if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
									OLED_ShowNumber(20,20,-robot.MOTOR_A.Target*1000,5,12);
	 else                 	        OLED_ShowString(15,20,"+"),
									OLED_ShowNumber(20,20, robot.MOTOR_A.Target*1000,5,12); 
		
	 if( robot.MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
									OLED_ShowNumber(75,20,-robot.MOTOR_A.Encoder*1000,5,12);
	 else                 	        OLED_ShowString(60,20,"+"),
									OLED_ShowNumber(75,20, robot.MOTOR_A.Encoder*1000,5,12);
	
	 //The fourth line of the display displays the content//
	 //显示屏第4行显示内容//
	 //Display the target speed and current speed of motor B
	 //显示电机B的目标速度和当前速度
	 OLED_ShowString(0,30,"R:");
	 if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
									OLED_ShowNumber(20,30,-robot.MOTOR_B.Target*1000,5,12);
	 else                 	        OLED_ShowString(15,30,"+"),
									OLED_ShowNumber(20,30,  robot.MOTOR_B.Target*1000,5,12); 
			
	 if( robot.MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
									OLED_ShowNumber(75,30,-robot.MOTOR_B.Encoder*1000,5,12);
     else                 	        OLED_ShowString(60,30,"+"),
									OLED_ShowNumber(75,30, robot.MOTOR_B.Encoder*1000,5,12);

	 //显示当前舵机PWM控制值
	 //Displays the current steering gear PWM control value
	 OLED_ShowString(00,40,"SERVO:");
	 
	 if( 1 == ServoState.UnLock ) //非自锁模式显示最后一次输出到舵机的PWM值
	 {
		OLED_ShowString(60,40," "),
		OLED_ShowNumber(80,40, ServoState.UnLock_Output,4,12); 
	 }
	 else //自锁模式下显示实时PWM值
	 {
		 if( robot.SERVO.Output<0)	OLED_ShowString(60,40,"-"),
									OLED_ShowNumber(80,40,-robot.SERVO.Output,4,12);
		 else                 	    OLED_ShowString(60,40,"+"),
									OLED_ShowNumber(80,40, robot.SERVO.Output,4,12); 
	 }

}


#elif defined DIFF_CAR
static void oled_diff_show(void)
{
	//显示加速度z轴数据
	OLED_ShowString(00,10,(const u8 *)"ACCEL ");
	oled_showfloat(imu.accel.z/1671.84f,80,10,2,2);
	
	//The third line of the display displays the content//
	//显示屏第3行显示内容//
	//Display the target speed and current speed of motor A
	//显示电机A的目标速度和当前速度	 
	OLED_ShowString(0,20,(const u8 *)"DL:");
	if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,20,(const u8 *)"-"),
	                            OLED_ShowNumber(20,20,-robot.MOTOR_A.Target*1000,5,12);
	else                 	    OLED_ShowString(15,20,(const u8 *)"+"),
	                            OLED_ShowNumber(20,20, robot.MOTOR_A.Target*1000,5,12); 

	if( robot.MOTOR_A.Encoder<0) OLED_ShowString(60,20,(const u8 *)"-"),
	                             OLED_ShowNumber(75,20,-robot.MOTOR_A.Encoder*1000,5,12);
	else                 	     OLED_ShowString(60,20,(const u8 *)"+"),
	                             OLED_ShowNumber(75,20, robot.MOTOR_A.Encoder*1000,5,12);

	//The fourth line of the display displays the content//
	//显示屏第4行显示内容//	
	//Display the target speed and current speed of motor B
	//显示电机B的目标速度和当前速度
	OLED_ShowString(0,30,(const u8 *)"DR:");
	if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,30,(const u8 *)"-"),
	                            OLED_ShowNumber(20,30,- robot.MOTOR_B.Target*1000,5,12);
	else                 	    OLED_ShowString(15,30,(const u8 *)"+"),
	                            OLED_ShowNumber(20,30,  robot.MOTOR_B.Target*1000,5,12); 

	if( robot.MOTOR_B.Encoder<0)OLED_ShowString(60,30,(const u8 *)"-"),
	                            OLED_ShowNumber(75,30,-robot.MOTOR_B.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,30,(const u8 *)"+"),
	                            OLED_ShowNumber(75,30, robot.MOTOR_B.Encoder*1000,5,12);
	
	OLED_ShowString(00,40,(const u8 *)"MA");
	if( robot.MOTOR_A.Output < 0 )   OLED_ShowString(20,40,(const u8 *)"-"),
	                                 OLED_ShowNumber(30,40,-robot.MOTOR_A.Output,4,12);
	else                 	         OLED_ShowString(20,40,(const u8 *)"+"),
	                                 OLED_ShowNumber(30,40, robot.MOTOR_A.Output,4,12); 
	OLED_ShowString(60,40,(const u8 *)"MB");
	if(robot.MOTOR_B.Output<0)       OLED_ShowString(80,40,(const u8 *)"-"),
	                                 OLED_ShowNumber(90,40,-robot.MOTOR_B.Output,4,12);
	else                 	         OLED_ShowString(80,40,(const u8 *)"+"),
	                                 OLED_ShowNumber(90,40, robot.MOTOR_B.Output,4,12);
	
}


#elif defined MEC_CAR || defined _4WD_CAR || defined OMNI_CAR
static void oled_mec_4wd_omni_show(void)
{
	//The second line of the display displays the content//
	//显示屏第2行显示内容//	
	//Display the target speed and current speed of motor A
	//显示电机A的目标速度和当前速度
	OLED_ShowString(0,10,"A");
	if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
								OLED_ShowNumber(20,10,-robot.MOTOR_A.Target*1000,5,12);
	else                 	    OLED_ShowString(15,10,"+"),
								OLED_ShowNumber(20,10, robot.MOTOR_A.Target*1000,5,12); 
	
	if( robot.MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
								OLED_ShowNumber(75,10,-robot.MOTOR_A.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,10,"+"),
								OLED_ShowNumber(75,10, robot.MOTOR_A.Encoder*1000,5,12);
	
	//The third line of the display displays the content//
	//显示屏第3行显示内容//	
	//Display the target speed and current speed of motor B
	//显示电机B的目标速度和当前速度
	OLED_ShowString(0,20,"B");		
	if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
								OLED_ShowNumber(20,20,-robot.MOTOR_B.Target*1000,5,12);
	else                 	    OLED_ShowString(15,20,"+"),
								OLED_ShowNumber(20,20, robot.MOTOR_B.Target*1000,5,12); 
	
	if( robot.MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
								OLED_ShowNumber(75,20,-robot.MOTOR_B.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,20,"+"),
								OLED_ShowNumber(75,20, robot.MOTOR_B.Encoder*1000,5,12);
	
	//The fourth line of the display displays the content//
	//显示屏第4行显示内容//
	//Display the target speed and current speed of motor C
	//显示电机C的目标速度和当前速度
	OLED_ShowString(0,30,"C");
	if( robot.MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
								OLED_ShowNumber(20,30,- robot.MOTOR_C.Target*1000,5,12);
	else                 	    OLED_ShowString(15,30,"+"),
								OLED_ShowNumber(20,30,  robot.MOTOR_C.Target*1000,5,12); 
		
	if( robot.MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
								OLED_ShowNumber(75,30,-robot.MOTOR_C.Encoder*1000,5,12);
	else                     	OLED_ShowString(60,30,"+"),
								OLED_ShowNumber(75,30, robot.MOTOR_C.Encoder*1000,5,12);
	
	
	//麦轮、四驱显示D电机
	#if !defined OMNI_CAR
	
	//Line 5 of the display displays the content//
	//显示屏第5行显示内容//
	OLED_ShowString(0,40,"D");
	if( robot.MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
								OLED_ShowNumber(20,40,- robot.MOTOR_D.Target*1000,5,12);
	else                 	    OLED_ShowString(15,40,"+"),
								OLED_ShowNumber(20,40,  robot.MOTOR_D.Target*1000,5,12); 
		
	if( robot.MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
								OLED_ShowNumber(75,40,-robot.MOTOR_D.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,40,"+"),
								OLED_ShowNumber(75,40, robot.MOTOR_D.Encoder*1000,5,12);
	
	//全向轮没有D电机,显示Z轴的速度
	#else
	
	OLED_ShowString(0,40,"MOVE_Z"); 
	oled_showfloat(robot_control.Vz,60,40,3,2);
	
	#endif
}


#endif


