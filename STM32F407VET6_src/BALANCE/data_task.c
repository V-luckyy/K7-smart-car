#include "data_task.h"
#include "apf.h"   /* g_apf_debug / g_apf_debug_valid: APF 0xFB debug uplink frame */

//���ڷ������ݵĽṹ��
SEND_DATA Send_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;

S21C_SensorData_t s21c_board = {
	.rangerA = 5,
	.rangerB = 5,
	.rangerC = 5,
	.rangerD = 5,
	.rangerE = 5,
	.rangerF = 5,
};

//����������
#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN 19
static uint8_t rangerbuffer[RangerFRAME_LEN];

/* ---- APF/Stanley debug uplink frame (0xFB), 27 bytes, sent @20Hz by Usart3_SendTask.
 *      1 header + 12 * int16(BE, x1000) + 1 BCC + 1 tail = 27.
 *      Only sent while g_apf_debug_valid==1 (i.e. APF task is actually running). */
#define APFDBG_HEAD 0xFB
#define APFDBG_TAIL 0x7D
#define APFDBG_LEN  27
#define APFDBG_NFIELD 12
static uint8_t apfdbgbuffer[APFDBG_LEN];

/**************************************************************************
Function: Robot Data Transmission Task: Sending robot status, IMU, speed, and other information to various interfaces.
Input   : none
Output  : none
�������ܣ� ���������ݷ�������,������ӿڷ��ͻ����˵�״̬,imu,�ٶȵ���Ϣ
��ڲ�������
����  ֵ����
**************************************************************************/
TaskHandle_t data_TaskHandle = NULL;

void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//��������20Hz��Ƶ������
			vTaskDelayUntil(&lastWakeTime, F2T(DATA_TASK_RATE));
			//Assign the data to be sent
			//��Ҫ���з��͵����ݽ��и�ֵ
			data_transition(); 
			Usart1_SendTask();
			Usart3_SendTask();
			CAN1_SendTask();
		}
}

/**************************************************************************
Functionality: Perform BCC (Block Check Character) verification on the input array and specified length
Input Parameters: Array start address, length to be verified
Return Value: BCC verification result
Author: WHEELTEC
�������ܣ�������������У��ĳ��Ƚ���BCCУ��
��ڲ����������׵�ַ,Ҫ����ĳ���
����  ֵ��bccУ����
��    �ߣ�WHEELTEC
**************************************************************************/
uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
static void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //֡β
	
	//���ݳ��͵Ĳ�ͬ�������Ӧ���˶�ѧ���⺯��,��ȡ������3������ٶ�
	float *vel;
	#if defined AKM_CAR || defined DIFF_CAR
		vel = Kinematics_akm_diff(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder);
	#elif defined MEC_CAR || defined _4WD_CAR
		vel = Kinematics_mec_4wd(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder,robot.MOTOR_D.Encoder);
	#elif defined OMNI_CAR
		vel = Kinematics_omni(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder);
	#endif
	
	//Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
	//�˶�ѧ���⣬�Ӹ����ֵ�ǰ�ٶ�������ᵱǰ�ٶ�
	Send_Data.Sensor_Str.Vel.X_speed = vel[0]*1000; //С��x���ٶ�,����1000������
	Send_Data.Sensor_Str.Vel.Y_speed = vel[1]*1000; //С��y���ٶ�,����1000������
	Send_Data.Sensor_Str.Vel.Z_speed = vel[2]*1000; //С��z���ٶ�,����1000������
	
	//The acceleration of the triaxial acceleration //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //���ٶȼ�Z��ת����ROS����Z��
	
	//The Angle velocity of the triaxial velocity //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	
	if( 0 == robot_control.FlagStop ) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //����������λʹ��״̬����ô��������Z����ٶ�
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;  
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
	Send_Data.Sensor_Str.Power_Voltage = robot.voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //֡ͷ
	Send_Data.buffer[1]=robot_control.FlagStop; //Car software loss marker //С������ʧ�ܱ�־λ
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//С�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[2]=Send_Data.Sensor_Str.Vel.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.Vel.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Vel.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Vel.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Vel.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Vel.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//��ص�ѹ,���Ϊ����8λ���ݷ���
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //����У��λ���㣬ģʽ1�Ƿ�������У��
	Send_Data.buffer[22]=Check_BCC(Send_Data.buffer,22); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //֡β
	
	///////////////////////�Զ��س���ر�����ֵ/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //֡ͷ��ֵ0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//֡β��ֵ0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)charger.ChargingCurrent;//��������ֵ
	
	Send_AutoCharge_Data.AutoCharge_Str.RED = charger.RED_STATE; //�����־λ��ֵ
	Send_AutoCharge_Data.AutoCharge_Str.Charging = charger.Charging; //�Ƿ��ڳ���־λ��ֵ

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//֡ͷ0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//��������8λ
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//��������8λ
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//�Ƿ���յ������־λ
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//�Ƿ��ڳ���־λ
	Send_AutoCharge_Data.buffer[5] = charger.AllowRecharge;									//�Զ��س��״̬
	Send_AutoCharge_Data.buffer[6] = Check_BCC(Send_AutoCharge_Data.buffer,6);				//У��λ
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//֡β0x7F
	///////////////////////�Զ��س���ر�����ֵ/////////////////////
	
	////////////// ����������֡ /////////////
	rangerbuffer[0] = RangerFRAME_HEAD;
	rangerbuffer[RangerFRAME_LEN-1] = RangerFRAME_TAIL;
	rangerbuffer[1] = (short)(s21c_board.rangerA*1000)>>8;
	rangerbuffer[2] = (short)(s21c_board.rangerA*1000);
	rangerbuffer[3] = (short)(s21c_board.rangerB*1000)>>8;
	rangerbuffer[4] = (short)(s21c_board.rangerB*1000);
	rangerbuffer[5] = (short)(s21c_board.rangerC*1000)>>8;
	rangerbuffer[6] = (short)(s21c_board.rangerC*1000);
	rangerbuffer[7] = (short)(s21c_board.rangerD*1000)>>8;
	rangerbuffer[8] = (short)(s21c_board.rangerD*1000);
	rangerbuffer[9] = (short)(s21c_board.rangerE*1000)>>8;
	rangerbuffer[10] = (short)(s21c_board.rangerE*1000);
	rangerbuffer[11] = (short)(s21c_board.rangerF*1000)>>8;
	rangerbuffer[12] = (short)(s21c_board.rangerF*1000);
	rangerbuffer[13] = 0;
	rangerbuffer[14] = 0;
	rangerbuffer[15] = 0;
	rangerbuffer[16] = 0;
	rangerbuffer[17] = Check_BCC(rangerbuffer,17);

	////////////// APF/Stanley debug frame (0xFB) /////////////
	if( g_apf_debug_valid )
	{
		int i;
		short f[APFDBG_NFIELD];
		f[0] = (short)(g_apf_debug.x        * 1000);
		f[1] = (short)(g_apf_debug.y        * 1000);
		f[2] = (short)(g_apf_debug.theta    * 1000);
		f[3] = (short)(g_apf_debug.v_cmd    * 1000);
		f[4] = (short)(g_apf_debug.w_cmd    * 1000);
		f[5] = (short)(g_apf_debug.w_stanley * 1000);
		f[6] = (short)(g_apf_debug.w_apf    * 1000);
		f[7] = (short)(g_apf_debug.v_act    * 1000);
		f[8] = (short)(g_apf_debug.w_act    * 1000);
		f[9] = (short)(g_apf_debug.dA * 1000);   /* m -> mm, 5000 = no obstacle */
		f[10] = (short)(g_apf_debug.dB * 1000);
		f[11] = (short)(g_apf_debug.dC * 1000);
		apfdbgbuffer[0] = APFDBG_HEAD;
		for(i = 0; i < APFDBG_NFIELD; i++)
		{
			apfdbgbuffer[1 + 2*i] = (uint8_t)(f[i] >> 8);
			apfdbgbuffer[2 + 2*i] = (uint8_t)(f[i]);
		}
		apfdbgbuffer[25] = Check_BCC(apfdbgbuffer, 25);   /* BCC over bytes 0..24 */
		apfdbgbuffer[26] = APFDBG_TAIL;
	}
	////////////// end APF debug frame /////////////
	////////////// ����������֡ /////////////
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
static void Usart1_SendTask(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		uart1_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart1_send(rangerbuffer[i]);
		}	
	}
	
	if(SysVal.HardWare_charger==1 || SysVal.HardWare_Ranger==1)
	{
		//���ڻس�װ��ʱ�����ϲ㷢���Զ��س���ر���
		for(i=0; i<8; i++)
		{
			uart1_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}

/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
�������ܣ�����3��������
��ڲ�������
����  ֵ����
**************************************************************************/
static void Usart3_SendTask(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		uart3_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart3_send(rangerbuffer[i]);
		}	
	}
	
	if(SysVal.HardWare_charger==1 || SysVal.HardWare_Ranger==1)
	{
		//���ڻس�װ��ʱ�����ϲ㷢���Զ��س���ر���
		for(i=0; i<8; i++)
		{
			uart3_send(Send_AutoCharge_Data.buffer[i]);
		}
	}

	/* APF/Stanley debug frame (0xFB) — sent @20Hz to K7 for tuning */
	if( g_apf_debug_valid )
	{
		for(i=0; i<APFDBG_LEN; i++)
		{
			uart3_send(apfdbgbuffer[i]);
		}
	}
}

/**************************************************************************
Function: CAN1 sends data
Input   : none
Output  : none
�������ܣ�CAN1��������
��ڲ�������
����  ֵ����
**************************************************************************/
static void CAN1_SendTask(void)
{
	u8 CAN_SENT[8],i;
	
	//24�ֽ����ݷ�3�鷢��,ʹ�ñ�׼֡id 0x101 0x102 0x103
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//���ڻس�װ��ʱ�����س�����������ݷ��͵��س�װ��
	if(SysVal.HardWare_charger==1)
	{
		CAN_Send_AutoRecharge();
	}
}

/*-------------------- Positive kinematics correlation function for each vehicle model ------------------------*/
/*--------------------------------         �������˶�ѧ������غ���          ------------------------------------*/
/**************************************************************************
Function Purpose: akm/diff Kinematic Analysis
Input Parameters: Left wheel speed, Right wheel speed, in meters per second (m/s)
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
�������ܣ�������/���� �˶�ѧ����
��ڲ����������ٶȡ������ٶ�,��λ m/s
����  ֵ��������x��y��z�����ٶ�
��    �ߣ�WHEELTEC
**************************************************************************/
#if defined AKM_CAR || defined DIFF_CAR
static float* Kinematics_akm_diff(float motorA,float motorB)
{
	static float vel[3];
	//xyz��������ٶ�
	vel[0] = (motorA + motorB)/2.0f;
	vel[1] = 0;
	vel[2] = (motorB - motorA)/robot.HardwareParam.WheelSpacing;
	
	return vel;
}

/**************************************************************************
Function Purpose: mec/4wd Kinematic Analysis
Input Parameters: Robot's four-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
�������ܣ�����/���� �˶�ѧ����
��ڲ������������ĸ��ֵ��ٶ�,��λ m/s
����  ֵ��������x��y��z�����ٶ�
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined MEC_CAR || defined _4WD_CAR
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD)
{
	static float vel[3];
	//xyz��������ٶ�
	vel[0] = (motorA+motorB+motorC+motorD)/4.0f;
	vel[1] = (motorA-motorB+motorC-motorD)/4.0f;
	vel[2] = (-motorA-motorB+motorC+motorD)/4.0f/( robot.HardwareParam.WheelSpacing + robot.HardwareParam.AxleSpacing );
	
	return vel;
}

/**************************************************************************
Function Purpose: omni Kinematic Analysis
Input Parameters: Robot's three-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
�������ܣ�ȫ���� �˶�ѧ����
��ڲ�����������3���ֵ��ٶ�,��λ m/s
����  ֵ��������x��y��z�����ٶ�
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined OMNI_CAR
static float* Kinematics_omni(float motorA,float motorB,float motorC)
{
	static float vel[3];
	//xyz��������ٶ�
	vel[0] = (motorC - motorB)/2.0f/robot.HardwareParam.X_PARAMETER;
	vel[1] = (motorA*2 - motorB - motorC)/3.0f;
	vel[2] = (motorA + motorB + motorC )/ 3.0f /robot.HardwareParam.TurnRadiaus;
	
	return vel;
}

#endif


