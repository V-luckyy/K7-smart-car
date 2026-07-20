/***********************************************
¹«Ë¾£ºÂÖÈ¤¿Æ¼¼£¨¶«İ¸£©ÓĞÏŞ¹«Ë¾
Æ·ÅÆ£ºWHEELTEC
¹ÙÍø£ºwheeltec.net
ÌÔ±¦µêÆÌ£ºshop114407458.taobao.com 
ËÙÂôÍ¨: https://minibalance.aliexpress.com/store/4455017
°æ±¾£ºV1.01
ĞŞ¸ÄÊ±¼ä£º2024-06-25

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.01
Update£º2024-06-25

All rights reserved
***********************************************/
#include "system.h"

//Task priority    //ÈÎÎñÓÅÏÈ¼¶
#define START_TASK_PRIO	1

//Task stack size //ÈÎÎñ¶ÑÕ»´óĞ¡	
#define START_STK_SIZE 	512  

//Task handle     //ÈÎÎñ¾ä±ú
TaskHandle_t StartTask_Handler;

TaskHandle_t g_reportErrTaskHandle = NULL;

//Task function   //ÈÎÎñº¯Êı
void start_task(void *pvParameters);
void ReportErrTask(void* param);




float Balance_Kp=295,Balance_Kd=1.48;
float Velocity_Kp=103.27,Velocity_Ki=0.56;
u16 PID_Parameter[10];
u16 Flash_Parameter[10];


#include "sensor_uart.h"       /* sensor_uart_init, g_sensor_dist_* */
#include "sensor_fusion.h"     /* APF_SensorFusion */
#include "apf.h"               /* apf_follow */
 #include "apf_sensor.h"

/* -- ??? -- */
CarState get_car_state(void);
CarState filter_car_state(CarState raw);

/* -- ?????? -- */
WayPoint waypoint_get_next(void);
void    wv_to_target(float v_cmd, float omega_cmd);

/* -- ??? (ROS-compatible) -- */
void ros_recv_target_callback(void);
void ros_send_car_state(CarState state, SensorObs obs);

/* -- ??? -- */
void update_oled(CarState state, WayPoint target, SensorObs obs);





//Main function //Ö÷º¯Êı
int main(void)
{ 
	systemInit(); //Hardware initialization //Ó²¼ş³õÊ¼»¯

	//Create the start task //´´½¨¿ªÊ¼ÈÎÎñ
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //ÈÎÎñº¯Êı
							(const char*    )"start_task",          //Task name       //ÈÎÎñÃû³Æ
							(uint16_t       )START_STK_SIZE,        //Task stack size //ÈÎÎñ¶ÑÕ»´óĞ¡
							(void*          )NULL,                  //Arguments passed to the task function //´«µİ¸øÈÎÎñº¯ÊıµÄ²ÎÊı
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //ÈÎÎñÓÅÏÈ¼¶
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //ÈÎÎñ¾ä±ú    					
	vTaskStartScheduler();  //Enables task scheduling //¿ªÆôÈÎÎñµ÷¶È	
							
							
							
	 while(1)
    {
        /* ================== ??? ================== */

        /*
         * 1.1 ??????????
         *
         *     ??? USART2/USART3/UART5 ??ISR?????,
         *     ?????????? g_sensor_dist_*?
         *
         *     ????????? (??? = 40.0 = ???)?
         */
        float dist_front = g_sensor_dist_front;
        float dist_left  = g_sensor_dist_left;
        float dist_right = g_sensor_dist_right;

        /* 1.2 ???? + IMU ???????? */
        CarState car_raw = get_car_state();

        /* 1.3 ???????, ???????? */
        CarState car_filt = filter_car_state(car_raw);


        /* ================== ?????? ================== */

        /*
         * 2.1 ???????
         *     ???: ros_recv_target_callback() ?CAN/?????
         *             ????WayPoint, ??????
         *     ???: waypoint_get_next() ??????????
         */
        WayPoint target = waypoint_get_next();

        /* 2.2 ????????? ? ?????????? */
        SensorObs obs = APF_SensorFusion(dist_front, dist_left, dist_right);

        /*
         * 2.3 APF ?????
         *
         *     ??:
         *         target    — ???
         *         car_filt  — ??????? (x,y,?,v,?)
         *         obs       — ???????? (??)
         *
         *     ??:
         *         v_cmd     — ????? (cm/s)
         *         omega_cmd — ????? (rad/s)
         */
        float v_cmd = 0.0f, omega_cmd = 0.0f;
        if(target.is_valid) {
					  APF_CircObs apf_obs[1];
            int obs_num = APF_SensorToCircObs(&obs, &car_filt, apf_obs, 1);
					 APF_Vec2 apf_target = {target.x, target.y};
					 APF_Car apf_car = {car_filt.x, car_filt.y, car_filt.theta};
            apf_follow(apf_target, &apf_car, apf_obs,obs_num,	 &v_cmd, &omega_cmd);
        }

        /*
         * 2.4 ?????: (v, ?) ? ???????
         *
         *     Target_Left  = v - omega * WHEEL_BASE_HALF
         *     Target_Right = v + omega * WHEEL_BASE_HALF
         *
         *     Target_Left/Target_Right ?????,
         *     5ms ISR ?? Incremental_PI ?????
         */
        wv_to_target(v_cmd, omega_cmd);


        /* ================== ??? ================== */

        /*
         * 3.1 ?????? + ???? ROS (CAN ID=0x100)
         *
         *     ????:
         *       ?1 (type=0x01): ???+??+?? (???????)
         *       ?2 (type=0x02): ???? (x,y,?,v)
         *       ?3 (type=0x03): ??? (x,y,???) — ?obs.valid???
         */
        ros_send_car_state(car_filt, obs);


        /* ================== ??? ================== */

        /* 4.1 OLED ?? */
        update_oled(car_filt, target, obs);


        /* ---- ???? ~20ms (50Hz) ---- */
        delay_ms(20);
    }
}
 
//Start task task function //¿ªÊ¼ÈÎÎñÈÎÎñº¯Êı
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //½øÈëÁÙ½çÇø
	
    //Create the task //´´½¨ÈÎÎñ
	xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //Ğ¡³µÔË¶¯¿ØÖÆÈÎÎñ
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL);  //User interaction tasks related to data display //ÓëÊı¾İÏÔÊ¾Ïà¹ØµÄÓÃ»§½»»¥ÈÎÎñ
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	 //LED light flashing task //LEDµÆÉÁË¸ÈÎÎñ
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     &data_TaskHandle); //Send data to each interface task
	if(SysVal.HardWare_Ver==V1_2)
	{
		xTaskCreate(D50A_Task,    "D50A_task",    D50A_STK_SIZE,    NULL, D50A_TASK_PRIO,    &d50a_TaskHandle); //D50A driver management task
	}
	if(SysVal.HardWare_Ver==V1_0) 	//IMU data read task //IMUÊı¾İ¶ÁÈ¡ÈÎÎñ,¸ù¾İ²»Í¬µÄÓ²¼ş°æ±¾Æô¶¯²»Í¬µÄÈÎÎñ.
	{
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
		xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      &show_TaskHandle);	 //Read the PS2 controller task //¶ÁÈ¡PS2ÊÖ±úÈÎÎñ
	}
		
	else if( SysVal.HardWare_Ver>=V1_1 )
	{
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	}

	  //Ğ¡³µ×Ô¼ìÉÏ±¨µ÷ÊÔÈÎÎñ
	  xTaskCreate(ReportErrTask,"ReportErrTask",128*4,NULL,osPriorityNormal,&g_reportErrTaskHandle);
	
    vTaskDelete(StartTask_Handler); //Delete the start task //É¾³ı¿ªÊ¼ÈÎÎñ

    taskEXIT_CRITICAL();            //Exit the critical section//ÍË³öÁÙ½çÇø
}






