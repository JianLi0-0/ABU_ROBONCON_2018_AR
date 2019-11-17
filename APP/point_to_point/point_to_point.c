#include "main.h"
#include "path.h"



/*********************pid�����******************************/
//#define angle_stop_threshold  (1.0f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
//#define pos_stop_threshold    (90.0f)   //�켣����Ŀ��������پ����л���ֵ

//#define angle_swith_threshold  (1.0f)   //�켣����Ŀ���Ƕ��л���ֵ
//#define pos_swith_threshold    (90.0f)  //�켣����Ŀ��������پ����л���ֵ
/***********************pid�����*****************************/


/*********************����pid�����******************************/
//#define angle_stop_threshold  (0.1f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
//#define pos_stop_threshold    (1.0f)   //�켣����Ŀ��������پ����л���ֵ

//#define angle_swith_threshold  (1.0f)   //�켣����Ŀ���Ƕ��л���ֵ
//#define pos_swith_threshold    (30.0f)  //�켣����Ŀ��������پ����л���ֵ
/***********************����pid�����*****************************/


#if ENCODER_ONLY
/*******************************/
#define angle_stop_threshold  (0.1f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
#define pos_stop_threshold    (20.0f)   //�켣����Ŀ��������پ����л���ֵ
#define angle_swith_threshold  (20.0f)   //�켣����Ŀ���Ƕ��л���ֵ
#define pos_swith_threshold    (400.0f)  //�켣����Ŀ��������پ����л���ֵ
/********************************/

#else

#define angle_stop_threshold  (0.2f)    //�켣����Ŀ���Ƕ�ֹͣ��ֵ
#define pos_stop_threshold    (40.0f)   //�켣����Ŀ��������پ����л���ֵ
#define angle_swith_threshold  (20.0f)   //�켣����Ŀ���Ƕ��л���ֵ
#define pos_swith_threshold    (400.0f)  //�켣����Ŀ��������پ����л���ֵ
#endif



//#define tracking_speed         3000
#define bezier_max             3000.0f
#define max_accel              100  //���޸ĳ�ʵ�ʼ��ٶ�
#define max_rotate_accel              500  //���޸ĳ�ʵ�ʼ��ٶ�
#define yaw_coefficience       10
//#define trapezoid_maximum      5000.0f //���μ����ٶ����ֵ

//#define trapezoid_minimum      500.0f
//#define accel_coefficient      0.2f  //λ��0.1Lʱ�ﵽ����ٶ�
/*************�趨ģʽ***********/
#define NOT_DEBUG_SINGLE_POINT 1


#define PARA_CURVE_DECELERATE 1
#define TRAPEZOID_DECELERATE 0

PID pid_yaw, pid_position, pid_position_fast;
u8 init = 1;
GOAL_ERRORS goal_errors;
int points  = 50 ;
int count = 0;
int debug_A,debug_B,debug_C,debug_D,debug_E;
trapezoid_param TRAPEZOID_PARAM;

float tracking_speed =0;
float cnt,p;

trapezoid_param TRAPEZOID_START_RELAY1={3000.0, 1000.0, 0.2}
							,TRAPEZOID_RELAY1_TZ1={2500,1000.0,0.2 }   //����������
							
							,TRAPEZOID_RELAY1_TZ2_1={3000,1000.0,0.3} //nonesense
							,TRAPEZOID_RELAY1_TZ2_2={3000,1000.0,0.3} //nonesense
							
							,TRAPEZOID_TZ2_RELAY2={3500,1000.0,0.2}
							,TRAPEZOID_RELAY2_TZ3={5000.0, 1000.0, 0.2}  //����������  //ԭ��4000
							,TRAPEZOID_STZ_RELAY2={3000, 1000.0, 0.2};

void TRACKING_TASK(struct POINT *POINTS)
{
	/***************************************************************************/
	/***************************************************************************/
	if(	LAST_TRACKING_STATE == TRACKING_UNKNOWN && TRACKING_STATE == TRACKING_GOING)
	{
		count = 0;
		switch(NOW_STATE)
	  {
		  case STZ_TO_RELAY1 :
				{
					points = (int)(sizeof(STARTZONE_TO_RELAY1)/sizeof(STARTZONE_TO_RELAY1[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.START_RELAY1;
					TRAPEZOID_PARAM = TRAPEZOID_START_RELAY1;
				}break;
			
		  case RELAY1_TO_TZ1 :
				{
					points = (int)(sizeof(RELAY1_TO_THROWZONE1)/sizeof(RELAY1_TO_THROWZONE1[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY1_TZ1;
					TRAPEZOID_PARAM = TRAPEZOID_RELAY1_TZ1;
				}break;

		  case TZ1_TO_RELAY1 :
				{
					points = (int)(sizeof(THROWZONE1_TO_RELAY1)/sizeof(THROWZONE1_TO_RELAY1[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY1_TZ1;
					TRAPEZOID_PARAM = TRAPEZOID_RELAY1_TZ1;
				}break;

		  case RELAY1_TO_TZ2 :       /******************δ��ɽ�����********************//******************δ��ɽ�����********************/
				{
					
					/*****************�˴����ã����ñ��������ߴ���******************/
					/*****************�˴����ã����ñ��������ߴ���******************/
					/*****************�˴����ã����ñ��������ߴ���******************/
					points = (int)(sizeof(RELAY1_TO_THROWZONE2)/sizeof(RELAY1_TO_THROWZONE2[0]));
				  if(count == 0)
					{
						goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY1_TZ2_1;
						TRAPEZOID_PARAM = TRAPEZOID_RELAY1_TZ2_1;
					}
					else if(count == 1)
					{
						goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY1_TZ2_2;
						TRAPEZOID_PARAM = TRAPEZOID_RELAY1_TZ2_2;
					}
					/******************δ��ɽ�����********************/
					/******************δ��ɽ�����********************/
					/******************δ��ɽ�����********************/

					/*****************�˴����ã����ñ��������ߴ���******************/
					/*****************�˴����ã����ñ��������ߴ���******************/
					/*****************�˴����ã����ñ��������ߴ���******************/
				}break;

		  case TZ2_TO_RELAY2 :
				{
					points = (int)(sizeof(THROWZONE2_TO_RELAY2)/sizeof(THROWZONE2_TO_RELAY2[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.TZ2_RELAY2;
					TRAPEZOID_PARAM = TRAPEZOID_TZ2_RELAY2;
				}break;

		  case RELAY2_TO_TZ3 :
				{
					points = (int)(sizeof(RELAY2_TO_THROWZONE3)/sizeof(RELAY2_TO_THROWZONE3[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY2_TZ3;
					TRAPEZOID_PARAM = TRAPEZOID_RELAY2_TZ3;
				}break;
				
			case RELAY2_TO_TZ2 :
				{
					points = (int)(sizeof(RELAY2_TO_THROWZONE2)/sizeof(RELAY2_TO_THROWZONE2[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.TZ2_RELAY2;
					TRAPEZOID_PARAM = TRAPEZOID_TZ2_RELAY2;
				}break;
		
		  case TZ3_TO_RELAY2 :
				{
					points = (int)(sizeof(THROWZONE3_TO_RELAY2)/sizeof(THROWZONE3_TO_RELAY2[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.RELAY2_TZ3;
					TRAPEZOID_PARAM = TRAPEZOID_RELAY2_TZ3;
				}break;
			case STZ_TO_RELAY2 :
				{
					points = (int)(sizeof(STARTZONE_TO_RELAY2)/sizeof(STARTZONE_TO_RELAY2[0]));
					goal_errors.distance_between_2pts = DISTANCE_BETWEEN_2PTS.START_RELAY2;
					TRAPEZOID_PARAM = TRAPEZOID_STZ_RELAY2;
				}break;

		  case STOP_STATE : {}break;	
				
		  default:{}
		}
	}
	/***************************************************************************/
	/***************************************************************************/
	
	if(TRACKING_STATE == TRACKING_GOING)
	{
		track_point(POINTS); //
		if(count >= points)
		{
			TRACKING_STATE = TRACKING_ARRIVED;
			count = points-1 ;
		}
	}
	else
	{
		ROBOT_STATE.Vx = ROBOT_STATE.Vy = ROBOT_STATE.Vz = 0;
	}
}


void tracking_init()
{
	PIDInit(&pid_yaw, 80.0, 0, 1.0, 1000.0, 1000.0); //-20.0
	PIDInit(&pid_position, 10.0 , 0.0 , 0.0, 2000, 2000);//   ����D����D����D����D����D����D����D
	PIDInit(&pid_position_fast, 25.0 , 0.0 , 2.0, 4000.0, 4000.0);
	
	
	/*******************����΢������**************************/
	RELAY1_TO_THROWZONE1[0].x += RELAY1_TO_THROWZONE1_OFFSET.x;
	RELAY1_TO_THROWZONE1[0].y += RELAY1_TO_THROWZONE1_OFFSET.y;
	RELAY1_TO_THROWZONE1[0].z += RELAY1_TO_THROWZONE1_OFFSET.z;
	
	RELAY1_TO_THROWZONE2[48].x += RELAY1_TO_THROWZONE2_OFFSET.x;
	RELAY1_TO_THROWZONE2[48].y += RELAY1_TO_THROWZONE2_OFFSET.y;
	RELAY1_TO_THROWZONE2[48].z += RELAY1_TO_THROWZONE2_OFFSET.z;
	
	RELAY2_TO_THROWZONE2[0].x += RELAY2_TO_THROWZONE2_OFFSET.x;
	RELAY2_TO_THROWZONE2[0].y += RELAY2_TO_THROWZONE2_OFFSET.y;
	RELAY2_TO_THROWZONE2[0].z += RELAY2_TO_THROWZONE2_OFFSET.z;
	
	RELAY2_TO_THROWZONE3[0].x += RELAY2_TO_THROWZONE3_OFFSET.x;
	RELAY2_TO_THROWZONE3[0].y += RELAY2_TO_THROWZONE3_OFFSET.y;
	RELAY2_TO_THROWZONE3[0].z += RELAY2_TO_THROWZONE3_OFFSET.z;
	/******************����΢������*******************************/
	
	
	
	/********************�����Ŀ���֮��ľ���****************************/
	
	DISTANCE_BETWEEN_2PTS.START_RELAY1 = sqrt(STARTZONE_TO_RELAY1[0].x*STARTZONE_TO_RELAY1[0].x + 
																						STARTZONE_TO_RELAY1[0].y*STARTZONE_TO_RELAY1[0].y);//4600.0; //
																						
	DISTANCE_BETWEEN_2PTS.RELAY1_TZ1 = sqrt((RELAY1_TO_THROWZONE1[0].x-STARTZONE_TO_RELAY1[0].x)*
																					(RELAY1_TO_THROWZONE1[0].x-STARTZONE_TO_RELAY1[0].x) + 
																				  (RELAY1_TO_THROWZONE1[0].y-STARTZONE_TO_RELAY1[0].y)*
																					(RELAY1_TO_THROWZONE1[0].y-STARTZONE_TO_RELAY1[0].y));//1800;  //2595.0;
																					
	DISTANCE_BETWEEN_2PTS.RELAY1_TZ2_1 = 19500.0;
	DISTANCE_BETWEEN_2PTS.RELAY1_TZ2_2 = 2695.0; 
	
	DISTANCE_BETWEEN_2PTS.TZ2_RELAY2 = sqrt((RELAY2_TO_THROWZONE2[0].x-THROWZONE2_TO_RELAY2[0].x)*
																					(RELAY2_TO_THROWZONE2[0].x-THROWZONE2_TO_RELAY2[0].x) + 
																				  (RELAY2_TO_THROWZONE2[0].y-THROWZONE2_TO_RELAY2[0].y)*
																					(RELAY2_TO_THROWZONE2[0].y-THROWZONE2_TO_RELAY2[0].y));//2495.0;
																					
	DISTANCE_BETWEEN_2PTS.RELAY2_TZ3 = sqrt((RELAY2_TO_THROWZONE3[0].x-THROWZONE3_TO_RELAY2[0].x)*
																					(RELAY2_TO_THROWZONE3[0].x-THROWZONE3_TO_RELAY2[0].x) + 
																				  (RELAY2_TO_THROWZONE3[0].y-THROWZONE3_TO_RELAY2[0].y)*
																					(RELAY2_TO_THROWZONE3[0].y-THROWZONE3_TO_RELAY2[0].y));//5755;
																					
	DISTANCE_BETWEEN_2PTS.START_RELAY2 = sqrt(STARTZONE_TO_RELAY2[0].x*STARTZONE_TO_RELAY2[0].x + 
																						STARTZONE_TO_RELAY2[0].y*STARTZONE_TO_RELAY2[0].y);//6639;
	
	/********************�����Ŀ���֮��ľ���****************************/	
	
	
	
	/********************�����Ŀ���֮��ľ���****************************/
	
	/********************�������߲�����ʼ��****************************/

	/********************�������߲�����ʼ��****************************/
	count = 0;
}


/*********************ֻ��ֹͣ��ʹ��pid����ֹͣ�㲻�ϸı��ٶȷ���***************************/
void track_point(POINT *POINTS)
{
	/**********�������********/
	
	goal_errors.x_error = POINTS[count].x - ROBOT_POSITION.X_POS;
	goal_errors.y_error = POINTS[count].y - ROBOT_POSITION.Y_POS;
	goal_errors.xy_error = ABSOLUTE(goal_errors.x_error) + ABSOLUTE(goal_errors.y_error);  //ȡ�����پ���ľ���ֵ
	
											/************************TZ3�Ҷ�Ѳ��**********************************/
											#if USING_Grayscale_Sensor   //��Ե�ֱ��PID
											if(NOW_STATE == RELAY2_TO_TZ3 || NOW_STATE == TZ3_TO_RELAY2)
											{
												goal_errors.y_error = 0;//Ѳ������Ľ����ٶ�
											}
											#endif
											/************************�Ҷ�Ѳ��***********************************/
	
	goal_errors.distance_error = sqrt(goal_errors.x_error*goal_errors.x_error + goal_errors.y_error*goal_errors.y_error);
	if(goal_errors.distance_error >= goal_errors.distance_between_2pts) 
		goal_errors.distance_error = goal_errors.distance_between_2pts;
	
	goal_errors.ran_distance = ABSOLUTE(goal_errors.distance_between_2pts - goal_errors.distance_error);
	goal_errors.z_error = POINTS[count].z - ROBOT_POSITION.ANGLE_POS;
	

	
	
	
	

	if( ABSOLUTE(goal_errors.z_error) < angle_swith_threshold )
	{
		if( ABSOLUTE(goal_errors.xy_error) < pos_swith_threshold)
		{
			if( POINTS[count].stop == 0 )
				count++;
			else if(POINTS[count].stop == 1)
			{
				if( ABSOLUTE(goal_errors.z_error) < angle_stop_threshold && 
					  ABSOLUTE(goal_errors.distance_error) < pos_stop_threshold )
				{
					count++;
				}
			}
		}
	}


	
	
	
	/***************��̬�ջ�******************/
	if(  ABSOLUTE(goal_errors.z_error) <angle_stop_threshold && ABSOLUTE(goal_errors.distance_error) < pos_stop_threshold)  //5-1���Ĺ�
	{
    ROBOT_STATE.Vz = 0;
	}
	else
	{
		PID_Position_Calc(&pid_yaw, ROBOT_POSITION.ANGLE_POS, POINTS[count].z);  //λ��ʽpid
	  ROBOT_STATE.Vz = pid_yaw.output;
	}
	
	/****************·������*************/
	if( ABSOLUTE(goal_errors.distance_error) < pos_stop_threshold && ABSOLUTE(goal_errors.z_error) <angle_stop_threshold)  //5-1���Ĺ�
	{
		ROBOT_STATE.Vx = 0;
		ROBOT_STATE.Vy = 0;
	}
	else
	{
		
		/*******����Ϊֹͣ�㣬ʹ��pid���٣�������ֹͣ�㣬��ֻ�ı��ٶȷ��򣬲��ı��С******/
		if(POINTS[count].stop == 0)
		{
		/***********************************�ȼ���**************************************************/
			cnt = count;
			p= points;
			if(cnt/p>0.90f)
			{
				tracking_speed = bezier_max * sqrt(p-cnt)/sqrt(0.10f*p);
			}
			else if(cnt/p<=0.30f)
			{
				tracking_speed = bezier_max * sqrt(cnt+1)/sqrt(0.30f*p);
			}
			else
			{
				tracking_speed = bezier_max;
			}
		/*************************************************************************************/
			ROBOT_STATE.Vx = tracking_speed * goal_errors.x_error / goal_errors.distance_error;  //tracking_speed * goal_errors.x_error / goal_errors.xy_error;  //
			ROBOT_STATE.Vy = tracking_speed * goal_errors.y_error / goal_errors.distance_error;  //tracking_speed * goal_errors.y_error / goal_errors.xy_error;  
		}
		else
		{

			if(NOW_STATE == RELAY1_TO_TZ2)
			{

			
			/*****************************PID**************************************/
			
			PID_Position_Calc(&pid_position, -goal_errors.distance_error, 0);
			ROBOT_STATE.Vx = pid_position.output* goal_errors.x_error / goal_errors.distance_error;
			ROBOT_STATE.Vy = pid_position.output* goal_errors.y_error/ goal_errors.distance_error;
							
			}
			
			
			else
			{
								/*************************�����ٶ�**********************************/
								/*******************б��trapezoid_maximum/(accel_coefficient*goal_errors.distance_between_2pts)**************/
								if(goal_errors.ran_distance <= (TRAPEZOID_PARAM.accel_coefficient*goal_errors.distance_between_2pts))  //���μ���
								{
									goal_errors.running_speed = TRAPEZOID_PARAM.trapezoid_minimum + ( goal_errors.ran_distance * TRAPEZOID_PARAM.trapezoid_maximum 
																											/ (TRAPEZOID_PARAM.accel_coefficient*goal_errors.distance_between_2pts));
								}
								else if(goal_errors.ran_distance >= (1-TRAPEZOID_PARAM.accel_coefficient)*goal_errors.distance_between_2pts
									      /*&& goal_errors.ran_distance <= 0.95f*goal_errors.distance_between_2pts*/)  //���μ���
								{
									goal_errors.running_speed =  goal_errors.distance_error * (TRAPEZOID_PARAM.trapezoid_maximum + TRAPEZOID_PARAM.trapezoid_minimum)\
																					 / (TRAPEZOID_PARAM.accel_coefficient*goal_errors.distance_between_2pts);
								}
								else                                                                            //��������ٶ�
								{
									goal_errors.running_speed = TRAPEZOID_PARAM.trapezoid_minimum + TRAPEZOID_PARAM.trapezoid_maximum;
								}
								ROBOT_STATE.Vx = goal_errors.running_speed * goal_errors.x_error / goal_errors.distance_error;
								ROBOT_STATE.Vy = goal_errors.running_speed * goal_errors.y_error/ goal_errors.distance_error;	
								
								
											/************************TZ3�Ҷ�Ѳ��**********************************/
											#if USING_Grayscale_Sensor   //��Ե�ֱ��PID
								

											if(NOW_STATE == RELAY2_TO_TZ3 || NOW_STATE == TZ3_TO_RELAY2)
											{
												dx=1;
												ROBOT_STATE.Vy = vy;//Ѳ������Ľ����ٶ�
											}
											xundo();
											#endif
											/************************�Ҷ�Ѳ��***********************************/
			}
		}
	}
}	




