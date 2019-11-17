#include "main.h"

/*****************************/
#define Wheel_Diameter 50.8f //mm
#define Encoder_Resolution 500 //mm
#define Delta_Time 0.005f //mm
#define Distance_per_line (PI*Wheel_Diameter/(Encoder_Resolution*4)) //mm
#define SIN45 0.70710678f //sin(pi/4) cos(pi/4)




#if DEBUG_ENCODER
int t3,t4,t8;
#endif

struct{
	float X_PROPORTION;
	float Y_PROPORTION;
	float ANGLE_OFFSET;
}COORDINATE_OFFSET = { 0.0, 0.0, 0.0};


Atti Attitude;
Enco Encoder;
ENCO_POS ROBOT_ENCO_POS;

/********************************/
ROBOT_POS ROBOT_POSITION = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
ROBOT_POS LIDAR_POSITION = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//float kalman_x,kalman_y,last_pos_x,delta_x;

u8 lidar_flag = 1;

void pos_update(void)
{
	Get_Robot_Possiton();
	/******************��¼�ϴ�����************************************/
	ROBOT_POSITION.LAST_X_POS = ROBOT_POSITION.X_POS;
	ROBOT_POSITION.LAST_Y_POS = ROBOT_POSITION.Y_POS;
	ROBOT_POSITION.LAST_ANGLE_POS = ROBOT_POSITION.ANGLE_POS;
	/******************��¼�ϴ�����************************************/
	
	#if ENCODER_ONLY
	ROBOT_POSITION.X_POS = ROBOT_ENCO_POS.X;  
	ROBOT_POSITION.Y_POS = ROBOT_ENCO_POS.Y;
	ROBOT_POSITION.ANGLE_POS = ROBOT_ENCO_POS.Z;
	#endif
	
	#if ENCODER_LIDAR_FUSION
	
	SEND_LIDAR_XY_MSG(CAN1, ROBOT_ENCO_POS.X, ROBOT_ENCO_POS.Y);
	SEND_LIDAR_ANGLE_MSG(CAN1, ROBOT_ENCO_POS.Z);
	if(Get_Lost_Error(MISSING_ERROR_LIDAR) == MISSING_ERROR_LIDAR)
	{
		printf("LIDAR ALREADY CAN MISSING \r\n");
	}
	
	
	
	
	
	if(TRACKING_STATE == TRACKING_GOING && NOW_STATE != RELAY1_TO_TZ2)
	{
		if(goal_errors.distance_error<400.0f)
		{
			lidar_flag = 0;
		}
		else
			lidar_flag = 1;
	}
	else if(TRACKING_STATE == TRACKING_GOING && NOW_STATE == RELAY1_TO_TZ2)
	{
		if(count>=47)
		{
			lidar_flag = 0;
		}
		else
			lidar_flag = 1;
	}
	else
	{
		lidar_flag = 1;
	}
	
	if(lidar_flag==1)
	{
		if(IS_LIDAR_RECEIVED)
		{
			ROBOT_ENCO_POS.X = LIDAR_POSITION.X_POS;   //  ʹ���״ﶨλ���ݸ���ȫ����λģ������
			ROBOT_ENCO_POS.Y = LIDAR_POSITION.Y_POS;
			ROBOT_ENCO_POS.Z = LIDAR_POSITION.ANGLE_POS;  //
			
			Attitude.YAW_OFFSET = Attitude.RAW_YAW - LIDAR_POSITION.ANGLE_POS;
			
			IS_LIDAR_RECEIVED = 0;
		}
	}
	else
	{
		IS_LIDAR_RECEIVED = 0;
	}
	
	
	
	
	
	ROBOT_POSITION.X_POS = ROBOT_ENCO_POS.X;  //
	ROBOT_POSITION.Y_POS = ROBOT_ENCO_POS.Y;
	ROBOT_POSITION.ANGLE_POS = ROBOT_ENCO_POS.Z;  //

	
//	if(ERROR_MSG[0]=='R' && ERROR_MSG[7]=='R')
//	{
//		SEND_LIDAR_XY_MSG(CAN1, ROBOT_ENCO_POS.X, ROBOT_ENCO_POS.Y);
//		ERROR_MSG[0] = ERROR_MSG[7] = 0;
//		printf("LIDAR GGGGGGGGGGGGGG \r\n");
//		ROBOT_POSITION.X_POS = ROBOT_ENCO_POS.X;  //ʵ�ʰ�װ��ԭ�趨������ת��180�ȣ��Ӹ���
//		ROBOT_POSITION.Y_POS = ROBOT_ENCO_POS.Y;
//		ROBOT_POSITION.ANGLE_POS = ROBOT_ENCO_POS.Z;
//	}
	
	#endif
	
	#if LIDAR_ONLY
		ROBOT_POSITION.X_POS = LIDAR_POSITION.X_POS;
		ROBOT_POSITION.Y_POS = LIDAR_POSITION.Y_POS;
		ROBOT_POSITION.ANGLE_POS = LIDAR_POSITION.ANGLE_POS;  //ʹ�������Ƿ��ؽǶ�
		
		Attitude.YAW_OFFSET = Attitude.RAW_YAW - LIDAR_POSITION.ANGLE_POS;
		
		SEND_LIDAR_XY_MSG(CAN1, ROBOT_ENCO_POS.X, ROBOT_ENCO_POS.Y);
		SEND_LIDAR_ANGLE_MSG(CAN1, ROBOT_ENCO_POS.Z);
	#endif
	
	/*********************��Ӳ���ֵ******************************/
	ROBOT_POSITION.X_POS += ROBOT_POSITION.X_POS_OFFSET ;
	ROBOT_POSITION.Y_POS += ROBOT_POSITION.Y_POS_OFFSET ;
	ROBOT_POSITION.ANGLE_POS += ROBOT_POSITION.ANGLE_POS_OFFSET ;
	/*********************��Ӳ���ֵ******************************/
	
}

/***********************************************************************/
/**************************��������λ����*******************************/
/***********************************************************************/

extern struct SGyro 	stcGyro;
extern struct SAngle 	stcAngle;

void LOCALIZATION_INIT()
{
	/**********��ȡ�����ʼ���Ƕ�*************************/
	int count = 0;
	while(count<100)
	{
		count++;
		delay_ms(5);
	}
	Attitude.YAW_OFFSET = (float)stcAngle.Angle[2]/32768*180;
//	encoder_Init_TIM3();
	encoder_Init_TIM4();
	encoder_Init_TIM8();
	/***********��ȡ�����ʼ���Ƕ�************************/
	
	/**********AR������Ϊԭ�㣬��ʼ��������λ��*************/
//	ROBOT_POSITION.X_POS_OFFSET = -565.0;
//	ROBOT_POSITION.Y_POS_OFFSET = 4565.0;
	ROBOT_POSITION.X_POS_OFFSET = 0.0;
	ROBOT_POSITION.Y_POS_OFFSET = 0.0;
	ROBOT_POSITION.ANGLE_POS_OFFSET = 0.0;
	/**********AR������Ϊԭ�㣬��ʼ��������λ��*************/
}

void Get_Robot_Possiton()
{
	//Get_Enconder();
	#if DEBUG_ENCODER
//	t3 += Read_Encoder(3);
	t4 += Read_Encoder(4);
	t8 += Read_Encoder(8);
	printf("t3:%d  t4:%d  t8:%d  \r\n",t3,t4,t8);
	#else
	Encoder.ENCONDER_SPEED_A = -Read_Encoder(4);
	Encoder.ENCONDER_SPEED_B = Read_Encoder(8);
	#endif
	
	Encoder.ENCONDER_POS_A += Encoder.ENCONDER_SPEED_A;
	Encoder.ENCONDER_POS_B += Encoder.ENCONDER_SPEED_B;
	Encoder.REAL_DELTA_POS_A = Encoder.ENCONDER_SPEED_A * Distance_per_line ;
	Encoder.REAL_DELTA_POS_B = Encoder.ENCONDER_SPEED_B * Distance_per_line ;
	
	//Attitude_Update();
	Attitude.RAW_YAW = (float)stcAngle.Angle[2]/32768*180;
	
	Attitude.YAW = Attitude.RAW_YAW - Attitude.YAW_OFFSET;
	
	Calculat_Odometry();
	//Orthogonal_Calculat_Odometry();
	
//	printf("%d \r\n",Encoder.ENCONDER_SPEED_A);
//	printf("%d \r\n",Encoder.ENCONDER_SPEED_B);
}



void Calculat_Odometry()
{
	ROBOT_ENCO_POS.Z  = Attitude.YAW; //convert to radian
	ROBOT_ENCO_POS.X += SIN45*cos(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B - Encoder.REAL_DELTA_POS_A) -
											SIN45*sin(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B + Encoder.REAL_DELTA_POS_A);
	ROBOT_ENCO_POS.Y += SIN45*sin(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B - Encoder.REAL_DELTA_POS_A) +
											SIN45*cos(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B + Encoder.REAL_DELTA_POS_A);
	
	
//	kalman_x += SIN45*cos(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B - Encoder.REAL_DELTA_POS_A) -
//											SIN45*sin(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B + Encoder.REAL_DELTA_POS_A);
//	kalman_y += SIN45*sin(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B - Encoder.REAL_DELTA_POS_A) +
//											SIN45*cos(ROBOT_ENCO_POS.Z/180.0f*PI)*(Encoder.REAL_DELTA_POS_B + Encoder.REAL_DELTA_POS_A);
	
}

void Orthogonal_Calculat_Odometry()
{
	ROBOT_ENCO_POS.Z  = Attitude.YAW; //convert to radian
	ROBOT_ENCO_POS.X += Encoder.REAL_DELTA_POS_B*cos(ROBOT_ENCO_POS.Z/180.0f*PI) - Encoder.REAL_DELTA_POS_A*sin(ROBOT_ENCO_POS.Z/180.0f*PI);
	ROBOT_ENCO_POS.Y += Encoder.REAL_DELTA_POS_B*sin(ROBOT_ENCO_POS.Z/180.0f*PI) + Encoder.REAL_DELTA_POS_A*cos(ROBOT_ENCO_POS.Z/180.0f*PI);
}


/***********************************************************************/
/**************************��������λ����*******************************/
/***********************************************************************/
