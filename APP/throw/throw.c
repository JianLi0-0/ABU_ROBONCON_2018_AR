#include "main.h"
#define angle_to_encoder   116.6667f   //77.78f  //  RE40 14:1  2000*14/360 77.78 ;; RE40 21:1 2000*21/360 =116.67 



/********************����˵��****************************************************************************************/
/*��NOW_STATE��TRACKING_STATE����watch1��ͶTZ1ʱ�򣬽�NOW_STATE���ó�2����Ͷ��׼������TRACKING_STATE���ó�0����Ͷ��
                                         ͶTZ2ʱ�򣬽�NOW_STATE���ó�4����Ͷ��׼������TRACKING_STATE���ó�0����Ͷ��
																				 ͶTZ3ʱ�򣬽�NOW_STATE���ó�6����Ͷ��׼������TRACKING_STATE���ó�0����Ͷ��*/
/********************************************************************************************************************/
TM_S THROW_MOTOR_STATE;
Throwing_M THROWING_MOTOR;
PID left_keep_pid,right_keep_pid,docking_pid,doublekill_docking_pid;
throwing_param TZ[5];
u8 double_kill = 0;
float error;
u8 num = 1;
u8 state = 1;

float init_angle=20;  //��ʼ��е�Ƕ�
long int initial_pos = 40;

void THROW_TASK()
{
	switch(NOW_STATE)
	{
		case THROW_ZONE1:
		{
			num = 1;
			throw_ball();
		}
		break;
		
		case THROW_ZONE2:
		{
//			printf("THROW_ZONE2 \r\n");
			num = 2;
			throw_ball();
		}
		break;
		
		case THROW_ZONE3:
		{
			num = 3;
			throw_ball();
		}
		break;
		
		case RELAY1_TO_TZ1 : 
		case RELAY1_TO_TZ2 : 
		case RELAY2_TO_TZ3 : 
		case RELAY2_TO_TZ2 :
		{
			DOCKING();
		}break;
		
		case RELAY_STATE1 : 
		{
			num = 1;
			THROWING_MOTOR.NUM = THROW_MOTOR_STATE.LEFT_NUM;
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //���ڽ���   //��ʱ��ʱ��RELAY_STATE == RELAY_SUCCEEDED����Ҫ���RELAY_STATE == RELAY_SUCCEEDED
			{
				DOCKING();
			}
			else
			{
				return_start_pos();
			}
		}
		break;
		
			
		case RELAY_STATE2 : 
		{
			num = 2;
			THROWING_MOTOR.NUM = THROW_MOTOR_STATE.RIGHT_NUM;
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)
			{
				DOCKING();
			}
			else
			{
				return_start_pos();
			}
		}
		break;
		
		case RELAY_STATE3 : 
		{
			num = 2;
			THROWING_MOTOR.NUM = THROW_MOTOR_STATE.RIGHT_NUM;
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)
			{
				DOCKING();
			}
			else
			{
				return_start_pos();
			}
		}
		break;	
		case RELAY_STATE4 : 
		{
			num = 3;
			double_kill = 1;    
			THROWING_MOTOR.NUM = THROW_MOTOR_STATE.RIGHT_NUM;
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)
			{
				DOCKING();
			}
			else
			{
				return_start_pos();
			}
		}
		break;
		default:
		{
			return_start_pos();
		}
		
	};
	#if DEBUG_THROW_MOTOR
	
	printf("pos: %ld   spd:%d \r\n",THROWING_MOTOR.Motor_pos,Real_Velocity_Value[0]);
	
	#endif
	
};
/*
	TZ[1].start_angle = 80 - init_angle;   //90����
	TZ[1].accelerate_angle = 100 - init_angle;  //����
	TZ[1].break_angle =144- init_angle;    //���ٽǶ�
	TZ[1].stop_angle = 190 - init_angle;    //
	TZ[1].start_position = TZ[1].start_angle * angle_to_encoder;
	TZ[1].accelerate_position = TZ[1].accelerate_angle * angle_to_encoder;
  TZ[1].break_position = TZ[1].break_angle * angle_to_encoder; 
  TZ[1].stop_position  = TZ[1].stop_angle * angle_to_encoder;
	TZ[1].throw_speed  = 2900;  //Ͷ���ٶ�             MAX:7200
	TZ[1].ACCELERATE = 900;     //���ٲ���ֵ
	TZ[1].DECELERATE = 300;     //����
	TZ[1].speed_after_decelerate = 100;  //ֱ�Ӹ�����ٶ�
*/


void THROW_TASK_INIT(void)
{
	initial_pos = (initial_pos - init_angle) * angle_to_encoder;
	PIDInit(&left_keep_pid,   1, 0.0, 15.0, 500, 500);
	PIDInit(&right_keep_pid,   1, 0.0, 15.0, 500, 500);
	PIDInit(&docking_pid, 1, 0.0, 20.0, 500, 500);
	PIDInit(&doublekill_docking_pid, 1, 0.0, 20.0, 500, 500);
	
	
	TZ[1].start_angle = 85 - init_angle;   //90����
	TZ[1].accelerate_angle = 100 - init_angle;  //����
	TZ[1].break_angle =140- init_angle;    //���ٽǶ�
	TZ[1].stop_angle = 240 - init_angle;    //
	/******************************�����������޸Ĳ���***************************************/
	TZ[1].start_position = TZ[1].start_angle * angle_to_encoder;
	TZ[1].accelerate_position = TZ[1].accelerate_angle * angle_to_encoder;
  TZ[1].break_position = TZ[1].break_angle * angle_to_encoder; 
  TZ[1].stop_position  = TZ[1].stop_angle * angle_to_encoder;
	/******************************�����������޸Ĳ���***************************************/
	TZ[1].throw_speed  = 3400;  //Ͷ���ٶ�             MAX:7200
	TZ[1].ACCELERATE = 900;     //���ٲ���ֵ
	TZ[1].DECELERATE = 300;     //����
	TZ[1].speed_after_decelerate = 500;  //ֱ�Ӹ�����ٶ�

	
//	TZ[1]=(throwing_param){90,100,180,240,2500,150,150,100}; //�����������
//	TZ[1]=(throwing_param){90,100,180,240,3000,200,200,100}; //�ɱ��������

	TZ[2].start_angle = 85 - init_angle;
	TZ[2].accelerate_angle = 80 - init_angle;
	TZ[2].break_angle = 170- init_angle;
	TZ[2].stop_angle = 220 - init_angle;
	/******************************�����������޸Ĳ���***************************************/
	TZ[2].start_position = TZ[2].start_angle * angle_to_encoder;
	TZ[2].accelerate_position = TZ[2].accelerate_angle * angle_to_encoder;
  TZ[2].break_position = TZ[2].break_angle * angle_to_encoder; 
  TZ[2].stop_position  = TZ[2].stop_angle * angle_to_encoder;
	/******************************�����������޸Ĳ���***************************************/
	TZ[2].throw_speed  = 3950;  //Ͷ���ٶ�
	TZ[2].ACCELERATE = 900;
	TZ[2].DECELERATE = 500;
	TZ[2].speed_after_decelerate = 200;
	
//	TZ[2]=(throwing_param){90,100,190,240,3000,150,100,100}; //�����������

//	TZ[3].start_angle = 80 - init_angle;
//	TZ[3].accelerate_angle = 130 - init_angle;
//	TZ[3].break_angle = 125 - init_angle;
//	TZ[3].stop_angle = 213 - init_angle;
//	/******************************�����������޸Ĳ���***************************************/
//	TZ[3].start_position = TZ[3].start_angle * angle_to_encoder;
//	TZ[3].accelerate_position = TZ[3].accelerate_angle * angle_to_encoder;
//  TZ[3].break_position = TZ[3].break_angle * angle_to_encoder; 
//  TZ[3].stop_position  = TZ[3].stop_angle * angle_to_encoder;
//	/******************************�����������޸Ĳ���***************************************/
//	TZ[3].throw_speed  = 5200;  //Ͷ���ٶ�
//	TZ[3].ACCELERATE = 900;
//	TZ[3].DECELERATE = 500;
//	TZ[3].speed_after_decelerate = 100;
//	

//	TZ[4].start_angle = 80 - init_angle;
//	TZ[4].accelerate_angle = 130 - init_angle;
//	TZ[4].break_angle = 105 - init_angle;
//	TZ[4].stop_angle = 200	- init_angle;
//	TZ[4].start_position = TZ[4].start_angle * angle_to_encoder;
//	TZ[4].accelerate_position = TZ[4].accelerate_angle * angle_to_encoder;
//  TZ[4].break_position = TZ[4].break_angle * angle_to_encoder; 
//  TZ[4].stop_position  = TZ[4].stop_angle * angle_to_encoder;
//	TZ[4].throw_speed  = 5100;  //Ͷ���ٶ�
//	TZ[4].ACCELERATE = 900;
//	TZ[4].DECELERATE = 500;
//	TZ[4].speed_after_decelerate = 50;

	
	
	
	
	TZ[3].start_angle = 80 - init_angle;
	TZ[3].accelerate_angle = 130 - init_angle;
	TZ[3].break_angle = 155 - init_angle;
	TZ[3].stop_angle = 213 - init_angle;
	/******************************?????????***************************************/
	TZ[3].start_position = TZ[3].start_angle * angle_to_encoder;
	TZ[3].accelerate_position = TZ[3].accelerate_angle * angle_to_encoder;
  TZ[3].break_position = TZ[3].break_angle * angle_to_encoder; 
  TZ[3].stop_position  = TZ[3].stop_angle * angle_to_encoder;
	/******************************?????????***************************************/
	TZ[3].throw_speed  = 3850;  //????
	TZ[3].ACCELERATE = 900;
	TZ[3].DECELERATE = 500;
	TZ[3].speed_after_decelerate = 500;
	

	TZ[4].start_angle = 80 - init_angle;
	TZ[4].accelerate_angle = 130 - init_angle;
	TZ[4].break_angle = 105.5 - init_angle;
	TZ[4].stop_angle = 213	- init_angle;
	TZ[4].start_position = TZ[4].start_angle * angle_to_encoder;
	TZ[4].accelerate_position = TZ[4].accelerate_angle * angle_to_encoder;
  TZ[4].break_position = TZ[4].break_angle * angle_to_encoder; 
  TZ[4].stop_position  = TZ[4].stop_angle * angle_to_encoder;
	TZ[4].throw_speed  = 6200;  //????
	TZ[4].ACCELERATE = 1150;
	TZ[4].DECELERATE = 800;
	TZ[4].speed_after_decelerate = 500;
	
	/*wendingban
	
		TZ[3].start_angle = 80 - init_angle;
	TZ[3].accelerate_angle = 130 - init_angle;
	TZ[3].break_angle = 155 - init_angle;
	TZ[3].stop_angle = 213 - init_angle;

	TZ[3].start_position = TZ[3].start_angle * angle_to_encoder;
	TZ[3].accelerate_position = TZ[3].accelerate_angle * angle_to_encoder;
  TZ[3].break_position = TZ[3].break_angle * angle_to_encoder; 
  TZ[3].stop_position  = TZ[3].stop_angle * angle_to_encoder;

	TZ[3].throw_speed  = 3950;  //????
	TZ[3].ACCELERATE = 900;
	TZ[3].DECELERATE = 500;
	TZ[3].speed_after_decelerate = 100;
	

	TZ[4].start_angle = 80 - init_angle;
	TZ[4].accelerate_angle = 130 - init_angle;
	TZ[4].break_angle = 106.5 - init_angle;
	TZ[4].stop_angle = 213	- init_angle;
	TZ[4].start_position = TZ[4].start_angle * angle_to_encoder;
	TZ[4].accelerate_position = TZ[4].accelerate_angle * angle_to_encoder;
  TZ[4].break_position = TZ[4].break_angle * angle_to_encoder; 
  TZ[4].stop_position  = TZ[4].stop_angle * angle_to_encoder;
	TZ[4].throw_speed  = 6200;  //????
	TZ[4].ACCELERATE = 1150;
	TZ[4].DECELERATE = 800;
	TZ[4].speed_after_decelerate = 50;
	
	*/
	
	
	THROW_MOTOR_STATE.LEFT_NUM = 1;
	THROW_MOTOR_STATE.RIGHT_NUM = 2;
	THROWING_MOTOR.NUM = THROW_MOTOR_STATE.LEFT_NUM;
}

u8 doublekill_end = 0;
void throw_ball(void)
{
	#if DEBUG_THROW_TASK
	TZ[1].start_position = TZ[1].start_angle * angle_to_encoder;
	TZ[1].accelerate_position = TZ[1].accelerate_angle * angle_to_encoder;
  TZ[1].break_position = TZ[1].break_angle * angle_to_encoder; 
  TZ[1].stop_position  = TZ[1].stop_angle * angle_to_encoder;
	
	TZ[2].start_position = TZ[2].start_angle * angle_to_encoder;
	TZ[2].accelerate_position = TZ[2].accelerate_angle * angle_to_encoder;
  TZ[2].break_position = TZ[2].break_angle * angle_to_encoder; 
  TZ[2].stop_position  = TZ[2].stop_angle * angle_to_encoder;
	
	TZ[3].start_position = TZ[3].start_angle * angle_to_encoder;
	TZ[3].accelerate_position = TZ[3].accelerate_angle * angle_to_encoder;
  TZ[3].break_position = TZ[3].break_angle * angle_to_encoder; 
  TZ[3].stop_position  = TZ[3].stop_angle * angle_to_encoder;
	
	TZ[4].start_position = TZ[4].start_angle * angle_to_encoder;
	TZ[4].accelerate_position = TZ[4].accelerate_angle * angle_to_encoder;
  TZ[4].break_position = TZ[4].break_angle * angle_to_encoder; 
  TZ[4].stop_position  = TZ[4].stop_angle * angle_to_encoder;
	#endif
	
	if(doublekill_end ==1 || num != 3)   //TZ3ʱ���������ҵ��
	{
					THROWING_MOTOR.Motor_pos = Real_Position_Value[THROWING_MOTOR.NUM-1];   //******************
					if( THROWING_MOTOR.Motor_pos > (TZ[num].start_position - 1000) && THROWING_MOTOR.Motor_pos <= TZ[num].break_position)
					{
						state = 1;
					}
					else if(THROWING_MOTOR.Motor_pos > TZ[num].break_position )
					{
						state = 2;
					}
					if(THROWING_MOTOR.Motor_pos > TZ[num].stop_position )
					{
						state = 3;
					}

					switch(state)
					{
						case 1: 
							THROWING_MOTOR.Motor_SPEED = limit_acceleration(THROWING_MOTOR.LAST_Motor_SPEED, TZ[num].throw_speed, TZ[num].ACCELERATE);break;
						case 2: 
							THROWING_MOTOR.Motor_SPEED = limit_acceleration(THROWING_MOTOR.LAST_Motor_SPEED, TZ[num].speed_after_decelerate, TZ[num].DECELERATE);break;
						case 3:
						{
							THROWING_MOTOR.Motor_SPEED = 0;
							if(double_kill)
							{
								THROW_STAGE = THROW_ENDED;
								doublekill_end = 0;
								double_kill = 0;            //Ͷ����ɣ��ر�˫��ģʽ
							}
							else
							{
								THROW_STAGE = THROW_ENDED;
							}
						}
						break;
						
						default:{THROWING_MOTOR.Motor_SPEED = 0; }
					}
				//	printf("tz: %d  state: %d  \r\n",num,state);
					THROWING_MOTOR.LAST_Motor_SPEED= THROWING_MOTOR.Motor_SPEED;
					CAN_RoboModule_DRV_Velocity_Mode(0, THROWING_MOTOR.NUM, 5000, THROWING_MOTOR.Motor_SPEED);  //******************��ʱ����2��
	}
	
	
	if(double_kill)
	{
	/*************************doublekill*****************************************/
			THROW_MOTOR_STATE.Motor_1eft_pos= Real_Position_Value[THROW_MOTOR_STATE.LEFT_NUM-1];
			if( THROW_MOTOR_STATE.Motor_1eft_pos > (TZ[4].start_position - 1000) && THROW_MOTOR_STATE.Motor_1eft_pos <= TZ[4].break_position)
			{
				THROW_MOTOR_STATE.Motor_1eft_SPEED = limit_acceleration(THROW_MOTOR_STATE.LAST_Motor_1eft_SPEED, TZ[4].throw_speed, TZ[4].ACCELERATE);
			}
			else if(THROW_MOTOR_STATE.Motor_1eft_pos > TZ[4].break_position )
			{
				THROW_MOTOR_STATE.Motor_1eft_SPEED = limit_acceleration(THROW_MOTOR_STATE.LAST_Motor_1eft_SPEED, TZ[4].speed_after_decelerate, TZ[4].DECELERATE);
			}
			if(THROW_MOTOR_STATE.Motor_1eft_pos > TZ[4].stop_position )
			{
				THROW_MOTOR_STATE.Motor_1eft_SPEED = 0;
				CAN_RoboModule_DRV_Velocity_Mode(0, THROW_MOTOR_STATE.LEFT_NUM, 5000, THROW_MOTOR_STATE.Motor_1eft_SPEED);
				
				
				
				/**************Ͷ����������delay*******************/
				static int delay_ms_record = 0;
				delay_ms_record+=1;
				if(delay_ms_record < 100)
					return;
				delay_ms_record = 0; //Artifiial_delay 1s
				/**************Ͷ����������delay*******************/
				
				
				
				doublekill_end = 1;
			}
			THROW_MOTOR_STATE.LAST_Motor_1eft_SPEED = THROW_MOTOR_STATE.Motor_1eft_SPEED;
			CAN_RoboModule_DRV_Velocity_Mode(0, THROW_MOTOR_STATE.LEFT_NUM, 5000, THROW_MOTOR_STATE.Motor_1eft_SPEED);
			
			

	}
	/**************************doublekill****************************************/
	
	#if DEBUG_ROBOT_BASE
		THROW_STAGE = THROW_ENDED;
	#endif
}
























void return_start_pos(void)
{
	THROW_MOTOR_STATE.Motor_1eft_pos= Real_Position_Value[THROW_MOTOR_STATE.LEFT_NUM-1];
	PID_Position_Calc(&left_keep_pid, THROW_MOTOR_STATE.Motor_1eft_pos, initial_pos); //�޸ĳ�����10��
	THROW_MOTOR_STATE.Motor_1eft_SPEED = left_keep_pid.output;
	CAN_RoboModule_DRV_Velocity_Mode(0, THROW_MOTOR_STATE.LEFT_NUM, 1000, THROW_MOTOR_STATE.Motor_1eft_SPEED);//pwm500,speed 500    //******************
	
	THROW_MOTOR_STATE.Motor_right_pos= Real_Position_Value[THROW_MOTOR_STATE.RIGHT_NUM-1];
	PID_Position_Calc(&right_keep_pid, THROW_MOTOR_STATE.Motor_right_pos, initial_pos); //�޸ĳ�����10��
	THROW_MOTOR_STATE.Motor_right_SPEED = right_keep_pid.output;
	CAN_RoboModule_DRV_Velocity_Mode(0, THROW_MOTOR_STATE.RIGHT_NUM, 1000, THROW_MOTOR_STATE.Motor_right_SPEED);//pwm500,speed 500    //******************
}
u8 relay_doublekill=0;
void DOCKING()
{
	THROWING_MOTOR.Motor_pos = Real_Position_Value[THROWING_MOTOR.NUM-1];
	PID_Position_Calc( &docking_pid, THROWING_MOTOR.Motor_pos, TZ[num].start_position);  //�ӳ�ʼ��λ������������λ��
	THROWING_MOTOR.Motor_SPEED = docking_pid.output;
	if( ABSOLUTE(THROWING_MOTOR.Motor_pos - TZ[num].start_position) < 50)  //�����ж�
	{
		THROWING_MOTOR.Motor_SPEED = 0;
		
		if(double_kill)
		{
			if(relay_doublekill==1)
			{
				RELAY_STATE = RELAY_SUCCEEDED;
				relay_doublekill = 0;
			}
		}
		else
		{
			RELAY_STATE = RELAY_SUCCEEDED;
		}

		
	}
	CAN_RoboModule_DRV_Velocity_Mode(0, THROWING_MOTOR.NUM, 1000, THROWING_MOTOR.Motor_SPEED);//pwm500,speed 500    //******************
	
	
	if(double_kill==1)
	{
		THROW_MOTOR_STATE.Motor_1eft_pos= Real_Position_Value[THROW_MOTOR_STATE.LEFT_NUM-1];
		PID_Position_Calc( &doublekill_docking_pid, THROW_MOTOR_STATE.Motor_1eft_pos, TZ[4].start_position);  //�ӳ�ʼ��λ������������λ��
		THROW_MOTOR_STATE.Motor_1eft_SPEED = doublekill_docking_pid.output;
		if( ABSOLUTE(THROW_MOTOR_STATE.Motor_1eft_pos - TZ[4].start_position) < 50)  //�����ж�
		{
			THROW_MOTOR_STATE.Motor_1eft_SPEED = 0;
			relay_doublekill = 1;
//			double_kill = 0;
		}
		CAN_RoboModule_DRV_Velocity_Mode(0, THROW_MOTOR_STATE.LEFT_NUM, 1000, THROW_MOTOR_STATE.Motor_1eft_SPEED);//pwm500,speed 500    //******************
	}
	
	
	#if DEBUG_ROBOT_BASE
	RELAY_STATE = RELAY_SUCCEEDED;
	#endif
	#if DEBUG_AUTOROBOT_NOESELF
	RELAY_STATE = RELAY_SUCCEEDED;
	#endif
	
}



