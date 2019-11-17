#include "main.h"
/*****************�궨�������������*********************/

#define BOTH_LED_TOGGLE() GPIO_ToggleBits(GPIOF, GPIO_Pin_9);GPIO_ToggleBits(GPIOF, GPIO_Pin_10)

/****************���������־********************
5-11 openmv �Ӿ��ӿ�

ײ����
#define angle_stop_threshold  (0.4f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
#define pos_stop_threshold    (15.0f)   //�켣����Ŀ��������پ����л���ֵ
#define angle_swith_threshold  (1.0f)   //�켣����Ŀ���Ƕ��л���ֵ
#define pos_swith_threshold    (800.0f)  //�켣����Ŀ��������پ����л���ֵ


ײ�� ��
	trapezoid_param TRAPEZOID_START_RELAY1={4500.0, 1500.0, 0.2}
							,TRAPEZOID_RELAY1_TZ1={5000,1000.0,0.2}
							,TRAPEZOID_RELAY1_TZ2_1={5000,1000.0,0.2}
							,TRAPEZOID_RELAY1_TZ2_2={5000,1000.0,0.2}
							,TRAPEZOID_TZ2_RELAY2={5000,1000.0,0.2}
							,TRAPEZOID_RELAY2_TZ3={8000.0, 1000.0, 0.2};						


5-14  ����ͦ�ȶ��� ��
#define angle_stop_threshold  (0.2f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
#define pos_stop_threshold    (10.0f)   //�켣����Ŀ��������پ����л���ֵ
#define angle_swith_threshold  (15.0f)   //�켣����Ŀ���Ƕ��л���ֵ
#define pos_swith_threshold    (900.0f)  //�켣����Ŀ��������پ����л���ֵ
#define tracking_speed         4300
#define max_accel              100  //���޸ĳ�ʵ�ʼ��ٶ�
#define max_rotate_accel              500  //���޸ĳ�ʵ�ʼ��ٶ�
trapezoid_param TRAPEZOID_START_RELAY1={4500.0, 1500.0, 0.2}
							,TRAPEZOID_RELAY1_TZ1={4000,1500.0,0.2}
							,TRAPEZOID_RELAY1_TZ2_1={4000,1500.0,0.2}
							,TRAPEZOID_RELAY1_TZ2_2={4000,1500.0,0.2}
							,TRAPEZOID_TZ2_RELAY2={4000,1500.0,0.2}
							,TRAPEZOID_RELAY2_TZ3={6500.0, 1500.0, 0.2};
	PIDInit(&pid_yaw, 80.0, 0.0, 1.0, 2000.0, 2000.0); //-20.0
	PIDInit(&pid_position, 10.0 , 0.0 , 0.0, tracking_speed, tracking_speed);//   ����D����D����D����D����D����D����D
	PIDInit(&pid_position_fast, 25.0 , 0.0 , 2.0, 4000.0, 4000.0);
****************���������־*********************/


/*5-20 if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //��ʱ��ʱ��RELAY_STATE == RELAY_SUCCEEDED����Ҫ���RELAY_STATE == RELAY_SUCCEEDED
       ���¼�ң��
			 

5-24 TZ3 Ŀ������   POINT RELAY2_TO_THROWZONE3_OFFSET = {-50.00, 0.0, 0, 1};
     17��54 ���Ͷ��ǰ��ʱ

5-25 б��

5-26 �ر�tim3������������ʹ��tim3 ch3��tim12 ch1 ���pwm������keyIO�����ļ�����ʼ�������࣬�ر�t9 pb14 ����
     ���ı��������߲���
  TZ1��Ͷб���ȶ�������
  TZ[1].start_angle = 85 - init_angle;   //90����
	TZ[1].break_angle =142- init_angle;    //���ٽǶ�
	TZ[1].stop_angle = 240 - init_angle;    //
	TZ[1].throw_speed  = 3500;  //Ͷ���ٶ�             MAX:7200
	TZ[1].ACCELERATE = 900;     //���ٲ���ֵ
	TZ[1].DECELERATE = 300;     //����
	TZ[1].speed_after_decelerate = 100;  //ֱ�Ӹ�����ٶ�
5-27 	PIDInit(&pid_position, 10.0 , 0.0 , 0.0, tracking_speed, tracking_speed/1.5);//   ����D����D����D����D����D����D����D

//								#if PARA_CURVE_DECELERATE
//								else if(goal_errors.ran_distance >= TRAPEZOID_PARAM.decel_coefficient*goal_errors.distance_between_2pts)  //���μ���
//								{
//										goal_errors.running_speed = (TRAPEZOID_PARAM.trapezoid_maximum+TRAPEZOID_PARAM.trapezoid_minimum)/( (TRAPEZOID_PARAM.decel_coefficient-1)*goal_errors.distance_between_2pts )
//																							*(goal_errors.ran_distance-TRAPEZOID_PARAM.decel_coefficient*goal_errors.distance_between_2pts) 
//																							*(goal_errors.ran_distance-TRAPEZOID_PARAM.decel_coefficient*goal_errors.distance_between_2pts) 
//																							+(TRAPEZOID_PARAM.trapezoid_maximum+TRAPEZOID_PARAM.trapezoid_minimum);
//								}
//								#endif

	
*/


u8 init_flag = 1;
u8 throw_switch = 1;
uint8_t Bsp_Int_Ok = 0;

ROBOT_WORKING_STATE LAST_STATE = PREPARE_STATE;
ROBOT_WORKING_STATE NOW_STATE = PREPARE_STATE;
Tracking_STATE TRACKING_STATE = TRACKING_UNKNOWN;
Tracking_STATE LAST_TRACKING_STATE = TRACKING_UNKNOWN;
Relay_STATE RELAY_STATE = RELAY_UNKNOWN;
Docking_STATE DOCKING_STATE = DOCKING_UNKNOWN;
/***************************************/
Throw_STATE THROW_STATE = THROW_UNKNOWN;
/***************************************/
Throw_STAGE THROW_STAGE = THROW_NOTKNOW;  //THROW_STATE
/***************************************/
POINT *NOW_TRAJECTORY;

void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
}

void BSP_Init(void)
{
	uart_init(115200);			//��ʼ�����ڲ�����Ϊ115200 
	USART2_Init(115200);
	GYRO_uart4_init(115200);
	Vision_uart5_init(115200);
	Action_uart_init(115200); 
	Timer2_Init(1000);//1ms..Timer2_Init(1000);
	Servo_Task_init();
	Nvic_Init();
//	LCD_PREPARE();
	THROW_TASK_INIT();
	LOCALIZATION_INIT();
	motor_init();
	IO_Initialization();
	LED_Init();
	tracking_init();
	remoter_init();
	/***********����ģʽѡ��************/
	delay_init(168);  //��ʼ����ʱ����
	BEEP_Init();
	KEY_Init();
	xunxian_Init();
	CHOOSE_MODE();
	/***********����ģʽѡ��************/
	Bsp_Int_Ok = 1;
}



ROBOT_WORKING_STATE get_robotstate(void)
{
	return NOW_STATE;
}

void set_robotstate(ROBOT_WORKING_STATE state)
{
	NOW_STATE = state;
}

void Finite_state_machine(void)
{
	#if DEBUG_ROBOT_BASE
	RELAY_STATE = RELAY_SUCCEEDED;
	THROW_STAGE = THROW_ENDED;
	#endif
	
	#if DEBUG_REMOTER
	#else
	THROW_STATE = THROW_SUCCEEDED;
	#endif
	
	LAST_STATE = NOW_STATE;
	LAST_TRACKING_STATE = TRACKING_STATE;
	
	switch( NOW_STATE )
	{
		case PREPARE_STATE : 
		{
//			if(Bsp_Int_Ok == 1)
//			{
//				NOW_STATE = STZ_TO_RELAY1;
//				NOW_TRAJECTORY = STARTZONE_TO_RELAY1;  // ��ʼ���¹켣
//				TRACKING_STATE = TRACKING_GOING;
//			}
			
			if(ZONE_TO_GO == 1 || ZONE_TO_GO == 2)   //TZ1��TZ2ʧ��
			{
				NOW_STATE = STZ_TO_RELAY1;
				NOW_TRAJECTORY = STARTZONE_TO_RELAY1;  // ��ʼ���¹켣
				TRACKING_STATE = TRACKING_GOING;
			}
			else if(ZONE_TO_GO == 3)
			{
				NOW_STATE = STZ_TO_RELAY2;						 //TZ3ʧ��
				NOW_TRAJECTORY = STARTZONE_TO_RELAY2;  // ��ʼ���¹켣
				TRACKING_STATE = TRACKING_GOING;
			}
			
		}
		break;
		
		case STZ_TO_RELAY1 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED /*&&LOCKING_STATE == LOCKING_SUCCEEDED*/)  //AA TP
			{
				if(ZONE_TO_GO == 1)
				{
					NOW_STATE = RELAY_STATE1;   //TZ1ʧ��
				}
				else if(ZONE_TO_GO == 2)
				{
					NOW_STATE = RELAY_STATE2;   //TZ2ʧ��
				}
				RELAY_STATE = RELAY_WAITING;
				TRACKING_STATE = TRACKING_UNKNOWN;
				/*&&LOCKING_STATE = LOCKING_UNKNOWN*/
			}
		}
		break;
		
		case STZ_TO_RELAY2 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
				NOW_STATE = RELAY_STATE4;  
				RELAY_STATE = RELAY_WAITING;
				THROW_STATE = THROW_UNKNOWN; /***************/
				TRACKING_STATE = TRACKING_UNKNOWN;
			}					
		}
		break;
		
		case RELAY1_TO_TZ1 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
				/**************Ͷ��ǰ����delay*******************/
				//Artifiial_delay(1000,100); ��ʱ1s
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < delay_before_throw)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ��ǰ����delay*******************/
				NOW_STATE = THROW_ZONE1;
				THROW_STAGE = THROWING;
				TRACKING_STATE = TRACKING_UNKNOWN;
				// HUIDU KEEP IN THIS POINT
			}					
		}
		break;
		
		case TZ1_TO_RELAY1 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED && THROW_STATE==THROW_SUCCEEDED) //AA TP && TZ1 THROW SUCCEEDED
			{
				NOW_STATE = RELAY_STATE2;  
				RELAY_STATE = RELAY_WAITING;
				THROW_STATE = THROW_UNKNOWN; /***************/
				TRACKING_STATE = TRACKING_UNKNOWN;
			}
			else if(TRACKING_STATE == TRACKING_ARRIVED && THROW_STATE==THROW_FAILED) //AA TP && TZ1 THROW FAILED
			{
				NOW_STATE = RELAY_STATE1;
				RELAY_STATE = RELAY_WAITING;
				THROW_STATE = THROW_UNKNOWN; /***************/
				TRACKING_STATE = TRACKING_UNKNOWN;
			}
		}
		break;

		case RELAY1_TO_TZ2 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP
			{
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
//				TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
				
				/**************����delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < delay_before_throw)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************����delay*******************/
				//Artifiial_delay(1000,100); ��ʱ1s
				NOW_STATE = THROW_ZONE2;
				THROW_STAGE = THROWING;
				TRACKING_STATE = TRACKING_UNKNOWN;
			}					
		}
		break;

		case TZ2_TO_RELAY2 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED && THROW_STATE == THROW_SUCCEEDED) //AA TP && TZ2 THROW SUCCEEDED
			{
				NOW_STATE = RELAY_STATE4;  
				RELAY_STATE = RELAY_WAITING;
				TRACKING_STATE = TRACKING_UNKNOWN;
				THROW_STATE = THROW_UNKNOWN;
			}		
			else if(TRACKING_STATE == TRACKING_ARRIVED && THROW_STATE==THROW_FAILED) //AA TP && TZ2 THROW FAILED
			{
				NOW_STATE = RELAY_STATE3;
				RELAY_STATE = RELAY_WAITING;
				TRACKING_STATE = TRACKING_UNKNOWN;
				THROW_STATE = THROW_UNKNOWN;
			}		
		}
		break;

		case RELAY2_TO_TZ3 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP
			{
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
//				TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
				
				/**************Ͷ��ǰ����delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < delay_before_throw)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				//Artifiial_delay(1000,100); //��ʱ1s
				/**************Ͷ��ǰ����delay*******************/
				NOW_STATE = THROW_ZONE3;
				THROW_STAGE = THROWING;
				TRACKING_STATE = TRACKING_UNKNOWN;
			}						
		}
		break;
		
		case RELAY2_TO_TZ2 :
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP
			{
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
//				TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
//				/**************Ͷ��ǰ���ɿ�����װ��*******************/
				
				/**************Ͷ��ǰ����delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < delay_before_throw)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ��ǰ����delay*******************/
				NOW_STATE = THROW_ZONE2;
				THROW_STAGE = THROWING;
				TRACKING_STATE = TRACKING_UNKNOWN;
			}
		}
		
		case TZ3_TO_RELAY2 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP
			{
				NOW_STATE = RELAY_STATE4;
				RELAY_STATE = RELAY_WAITING;
				TRACKING_STATE = TRACKING_UNKNOWN;
				THROW_STATE = THROW_UNKNOWN;
			}						
		}
		break;
		
		
		case THROW_ZONE1 : 
		{
			if(THROW_STAGE == THROW_ENDED)   //THROW ENDED
			{
				NOW_STATE = TZ1_TO_RELAY1;
				THROW_STATE = THROW_STATE_WAITING; //���ｻ����ǰ����״̬�ȴ�
				NOW_TRAJECTORY = THROWZONE1_TO_RELAY1;
				TRACKING_STATE = TRACKING_GOING;
				THROW_STAGE = THROW_NOTKNOW;
			}	
		}
		break;

		case THROW_ZONE2 : 
		{
			if(THROW_STAGE == THROW_ENDED)   //THROW ENDED
			{
				NOW_STATE = TZ2_TO_RELAY2;
				THROW_STATE = THROW_STATE_WAITING; //���ｻ����ǰ����״̬�ȴ�
				NOW_TRAJECTORY = THROWZONE2_TO_RELAY2;
				TRACKING_STATE = TRACKING_GOING;
				THROW_STAGE = THROW_NOTKNOW;
			}		
		}
		break;

		case THROW_ZONE3 : 
		{
			if(THROW_STAGE == THROW_ENDED)   //THROW ENDED
			{
				NOW_STATE = TZ3_TO_RELAY2;
				THROW_STATE = THROW_STATE_WAITING; //���ｻ����ǰ����״̬�ȴ�
				NOW_TRAJECTORY = THROWZONE3_TO_RELAY2;
				TRACKING_STATE = TRACKING_GOING;
				THROW_STAGE = THROW_NOTKNOW;
			}					
		}
		break;		
		
		case RELAY_STATE1 : 
		{
			if(RELAY_STATE == RELAY_SUCCEEDED)  //RELAY SUCCEEDED  ǰ��Ͷ��1
			{
				/**************Ͷ����������delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < 500)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ����������delay*******************/
				NOW_STATE = RELAY1_TO_TZ1;
				NOW_TRAJECTORY = RELAY1_TO_THROWZONE1;
				TRACKING_STATE = TRACKING_GOING;
				RELAY_STATE = RELAY_UNKNOWN;
			}			
		}
		break;	
		
		case RELAY_STATE2 : 
		{
			if(RELAY_STATE == RELAY_SUCCEEDED)  //RELAY SUCCEEDED  ǰ��Ͷ��2
			{
				/**************Ͷ����������delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < 1000)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ����������delay*******************/
				NOW_STATE = RELAY1_TO_TZ2;
				NOW_TRAJECTORY = RELAY1_TO_THROWZONE2;
				RELAY_STATE = RELAY_UNKNOWN;
				TRACKING_STATE = TRACKING_GOING;
			}					
		}
		break;
		
		case RELAY_STATE3 : 
		{
			if(RELAY_STATE == RELAY_SUCCEEDED)  //RELAY SUCCEEDED  ǰ��Ͷ��2
			{
				/**************Ͷ����������delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < 500)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ����������delay*******************/
				NOW_STATE = RELAY2_TO_TZ2;
				NOW_TRAJECTORY = RELAY2_TO_THROWZONE2;
				RELAY_STATE = RELAY_UNKNOWN;
				TRACKING_STATE = TRACKING_GOING;
			}
			
		}
		break;	
		
		case RELAY_STATE4 : 
		{
			if(RELAY_STATE == RELAY_SUCCEEDED)  //RELAY SUCCEEDED  ǰ��Ͷ��3
			{
				/**************Ͷ����������delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < 500)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ����������delay*******************/
				NOW_STATE = RELAY2_TO_TZ3;
				NOW_TRAJECTORY = RELAY2_TO_THROWZONE3;
				RELAY_STATE = RELAY_UNKNOWN;
				TRACKING_STATE = TRACKING_GOING;
			}				
		}
		break;	
		
		case STOP_STATE : 
		{
//			printf("STOP_STATE\r\n");
		}
		break;	
		
		
	}
	
	
	
	
	
	
	
	/*
	switch( NOW_STATE )
	{
	
		case PREPARE_STATE : 
		{

		}
		break;
		
		case STZ_TO_RELAY1 : 
		{

		}
		break;

		case RELAY1_TO_TZ1 : 
		{
				
		}
		break;

		case TZ1_TO_RELAY1 : 
		{

		}
		break;

		case RELAY1_TO_TZ2 : 
		{

		}
		break;

		case TZ2_TO_RELAY2 : 
		{

		}
		break;

		case RELAY2_TO_TZ3 : 
		{
	
		}
		break;
		
		case RELAY2_TO_TZ2 :
		{

		}
		
		case TZ3_TO_RELAY2 : 
		{
		
		}
		break;
		
		
		case THROW_ZONE1 : 
		{

		}
		break;

		case THROW_ZONE2 : 
		{

		}
		break;

		case THROW_ZONE3 : 
		{

		}
		break;		
		
		case RELAY_STATE1 : 
		{

		}
		break;	
		
		case RELAY_STATE2 : 
		{

		}
		break;	
		
		case RELAY_STATE3 : 
		{
			
		}
		break;	
		
		case RELAY_STATE4 : 
		{

					
		}
		break;	
		
		case STOP_STATE : 
		{
			
		}
		break;	
		
	}
	
	*/
}



int tempcount=1;
float a,b,c;

void Task_1000HZ(void)
{
	Supervise();
  Finite_state_machine();
	TRACKING_TASK(NOW_TRAJECTORY);  //can be replaced with tracking_sensors_task
	//LOCKED_IN_POINT_TASK();
	TEST_REMOTER_CHANGE_STATE();
	base_control_task();//���Ը�Ƶ��
}

void Task_500HZ(void)
{

}

void Task_250HZ(void)
{
	
}
void Task_200HZ(void)
{
	THROW_TASK();  //���ټ��ٿ��ǿ���Ƶ��//���ټ��ٿ��ǿ���Ƶ��//���ټ��ٿ��ǿ���Ƶ��//���ټ��ٿ��ǿ���Ƶ��
}

void Task_100HZ(void)
{
	pos_update(); //����Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ�ʸ���Ƶ��
//	a = MOTOR_STATE.Motor_B_SPEED;
//	b = MOTOR[1].inner_rpm;
//	c = M3508_pid[1].output;
//	scope(a,b,c);
}

void Task_50HZ(void)
{
//	send_pos(ROBOT_ENCO_POS.X, ROBOT_ENCO_POS.Y, ROBOT_ENCO_POS.Z);
	EXCEPTION_HANDLING();	
}

void Task_20HZ(void)
{
	Servo_Task(); //20ms  �������

	#if DEBUG_HARDWARE
	if(Get_Lost_Error(MISSING_ERROR_GYRO) == MISSING_ERROR_GYRO)
	{
		printf("GYRO_GG\r\n");
		NOW_STATE = STOP_STATE;
	}
//	#if LIDAR_ONLY
//	if(Get_Lost_Error(MISSING_ERROR_LIDAR) == MISSING_ERROR_LIDAR)
//	{
//		printf("LIDAR_GG\r\n");
//		NOW_STATE = STOP_STATE;
//	}
//	#endif
	if(Get_Lost_Error(MISSING_ERROR_MOTOR4) == MISSING_ERROR_MOTOR4)
	{
		printf("THROWMOTOR1_GG\r\n");
		NOW_STATE = STOP_STATE;
	}
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR5) == MISSING_ERROR_MOTOR5)
//		printf("LIDAR_GG\r\n");
	if(Get_Lost_Error(MISSING_ERROR_MOTOR1) == MISSING_ERROR_MOTOR1)
	{
		printf("BASE_MOTOR1_GG\r\n");
		NOW_STATE = STOP_STATE;
	}
	if(Get_Lost_Error(MISSING_ERROR_MOTOR2) == MISSING_ERROR_MOTOR2)
	{
		printf("BASE_MOTOR2_GG\r\n");
		NOW_STATE = STOP_STATE;
	}
	if(Get_Lost_Error(MISSING_ERROR_MOTOR3) == MISSING_ERROR_MOTOR3)
	{
		printf("BASE_MOTOR3_GG\r\n");
		NOW_STATE = STOP_STATE;
	}
	#endif

	#if DEBUG_LIDAR_POS
  printf("  %f  %f   %f,    %d    %d   %d     \r\n", LIDAR_POSITION.X_POS,LIDAR_POSITION.Y_POS,LIDAR_POSITION.ANGLE_POS , ROBOT_STATE.Vx, ROBOT_STATE.Vy, ROBOT_STATE.Vz); //  //
	#endif
	
	#if DEBUG_POS_AND_SPD
  printf("  %f  %f   %f,    %d    %d   %d     \r\n", ROBOT_ENCO_POS.X,ROBOT_ENCO_POS.Y,ROBOT_ENCO_POS.Z , ROBOT_STATE.Vx, ROBOT_STATE.Vy, ROBOT_STATE.Vz); //  //
	#endif
}

void Task_10HZ(void)
{

}

void Task_5HZ(void)
{
  THROWING_SHINING_TASK();
}

void Task_1HZ(void)
{
	BOTH_LED_TOGGLE();
}

