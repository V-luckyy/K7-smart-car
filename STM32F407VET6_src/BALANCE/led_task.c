#include "led_task.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

void RGBControl_task(void *pvParameters);

/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
	#if defined DIFF_CAR
	//灯带车型,使用灯带任务,不使用此任务
	if( robot.type == 5 )
	{
		xTaskCreate(RGBControl_task, "rgb_task", LED_STK_SIZE*2,      NULL, LED_TASK_PRIO,      NULL);
		vTaskDelete(NULL);
	}
	#elif defined MEC_CAR
	if( robot.type == 9 )
	{
		xTaskCreate(RGBControl_task, "rgb_task", LED_STK_SIZE*2,      NULL, LED_TASK_PRIO,      NULL);
		vTaskDelete(NULL);
	}
	#endif
	
	
    while(1)
    {
        static u8 led_state=0;//LED亮灭设置
		
		//检测自动回充状态,对LED灯颜色进行修改
		static u8 check_charger_mode;
		static u8 check_Charging;
		
		//进入回充状态,LED颜色为黄色
		if( 0 == check_charger_mode && 1 == charger.AllowRecharge ) LED_SetColor(LED_Yellow);
		
		//退出回充状态,LED颜色设置默认颜色
		if( 1 == check_charger_mode && 0 == charger.AllowRecharge ) LED_SetColor(LED_Red);
		check_charger_mode = charger.AllowRecharge;
		
		if( 1==charger.AllowRecharge ) 
		{	//自动回充状态下充电成功时将LED设置为紫色
			if( 0==check_Charging && 1 == charger.Charging ) LED_SetColor(LED_Purple);
			if( 1==check_Charging && 0 == charger.Charging ) LED_SetColor(LED_Yellow);
			check_Charging = charger.Charging; 
		}

        //LED闪烁任务
        led_state = !led_state;
        LED_SetState(led_state);
		
        //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function
        //LED闪烁任务非常简单，对频率精度要求低，使用相对延时函数
        vTaskDelay(SysVal.LED_delay);
    }
}


//灯带控制优先级由高到低
enum{
	RGB_Charging = 0,                //充电中
	RGB_FindChargingStations = 1,    //寻找充电桩
	RGB_LowPower = 2,                //电量低
	RGB_RangerWarning = 3,           //超声波识别障碍物警示
	RGB_UserDefined = 4,             //用户自定义
	RGB_Default = 5                  //默认状态
};

//用户自定义RGB状态
uint8_t userdefine_rgb[4]={ 0 };

//充电指示灯标志位
static uint8_t charging_color=0;

static uint8_t RGB_PriorityManagement(void)
{
	//自动回充相关标志位
	static uint32_t charging_green=0;
	static uint32_t charging_red=0;
	
	//自动回充判定
	if( charger.AllowRecharge == 1 ) 
	{
		//充电两种状态使用标志位切换
		if( charger.Charging==0 ) 
		{
			charging_green=0;
			charging_red=0;
			charging_color=0;//未充电时,清空相关标志位
		}
		else if( charger.Charging==1 && charger.ChargingCurrent < 0.65f )
		{
			charging_green++;
			charging_red=0;
			if( charging_green > 5*5 ) charging_color = 1;//绿灯常亮
		}
		else if( charger.Charging==1 && charger.ChargingCurrent >= 0.65f )
		{
			charging_green=0;
			charging_red++;
			if( charging_red > 5*2 ) charging_color = 2;//红灯常亮
		}
		
		//模拟调试
//		if( g_sys_debug.chargingfull==1 )
//		{
//			ChargeDev.ChargingFlag=1;
//			charging_green++;
//			charging_red=0;
//			if( charging_green > 5*5 ) charging_color = 1;//绿灯常亮
//		}
//		else if( g_sys_debug.chargingON==1 )
//		{
//			ChargeDev.ChargingFlag=1;
//			charging_green=0;
//			charging_red++;
//			if( charging_red > 5*2 ) charging_color = 2;//红灯常亮
//		}
		
		//返回充电中或寻找充电桩状态
		if( charger.Charging==1 ) return RGB_Charging; 
		else return RGB_FindChargingStations;
	}
	
	//电量低判断
	if( robot.LowPower ) return RGB_LowPower; 
	
	//超声波警示判断
	if( s21c_board.rangerA < 0.5f ||s21c_board.rangerB  < 0.5f || s21c_board.rangerC  < 0.5f ||
		s21c_board.rangerD < 0.5f || s21c_board.rangerE < 0.5f || s21c_board.rangerF < 0.5f )
	{
		return RGB_RangerWarning;
	}
	
	//用户自定义
	if( userdefine_rgb[0]==1 ) 
	{
		return RGB_UserDefined;
	}
	
	//默认状态
	return RGB_Default;

}

void RGBControl_task(void *pvParameters)
{
	const uint16_t TaskFreq = 5;
	
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//RGB灯带对象
	pRGBLightInterface_t rgb = &UserRGBLight;
	
	//实时状态机
	uint8_t RGB_Sm = RGB_Default;
	
	//灯带初始化与自检
	rgb->init();
	rgb->SetColor(255,0,0);
	vTaskDelay(1000);
	rgb->SetColor(0,255,0);
	vTaskDelay(1000);
	rgb->SetColor(0,0,255);
	vTaskDelay(1000);
	
	while(1)
	{
		//获取RGB状态灯状态
		RGB_Sm = RGB_PriorityManagement();
		
		switch( RGB_Sm )
		{
			case RGB_Charging:
				//即将充满为绿色,充电中为红色
				if( charging_color==2 )
				{
					rgb->SetColorFade(32,0,0);
				}
				else if( charging_color==1 )
				{
					rgb->SetColorFade(0,32,0);
				}
				break;
			case RGB_FindChargingStations:
				//紫灯常亮
				rgb->SetColorFade(100,0,128);
				break;
			case RGB_LowPower:
				//紫灯低频闪烁
				rgb->SetBlink(100,0,128,2000);
				break;
			case RGB_RangerWarning:
				//黄灯警示
				rgb->SetColorFade(255,128,0);
				break;
			case RGB_UserDefined:
				//用户自定义
				rgb->SetColorFade(userdefine_rgb[1],userdefine_rgb[2],userdefine_rgb[3]);
				break;
			case RGB_Default:
				//设置灯带默认状态
				rgb->turnoff();
				break;
			default:
				break;
		}	
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}




