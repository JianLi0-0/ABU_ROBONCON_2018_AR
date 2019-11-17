#include "main.h"

#define robot_coordinate 0
#define motor_USEING_RE40 1
#define motor_USEING_M3508 1
#define MAXIMUM_ANGULAR_SPEED 1000
#define MAXIMUM_LINEAR_SPEED 7000

R_S ROBOT_STATE;
M_S MOTOR_STATE;

//float Px,Py,Pz;
//float PVx,PVy,PVz;
//long int wheel_speed[4];

#if DEBUG_M3508_PID_PARAM
int16_t m_target1,m_target2,m_target3,m_target4;
#endif
PID M3508_pid[4];
extern Motor MOTOR[4];

extern ROBOT_POS ROBOT_POSITION;


/***S型加减速***/
//int sigmoid_count;
//float sigmoid[21]={0.018,0.029 ,0.047 , 0.076 , 0.119 , 0.182 , 0.269 , 
//	                 0.377  , 0.500 , 0.622 ,0.731 , 0.817 , 0.881, 0.924 ,
//                 	0.952 , 0.970 ,0.982 , 0.989 , 0.993 , 0.996 , 0.997};

/*********机器人坐标系逆运动学************/
void Kinematic_Analysis(float Vx, float Vy, float Vz)
{
	MOTOR_STATE.Motor_A_SPEED   = Vx + L_PARAMETER*Vz;
	MOTOR_STATE.Motor_B_SPEED   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	MOTOR_STATE.Motor_C_SPEED   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}

/*********世界坐标系逆运动学*************/
void World_Kinematic_Analysis(float Vx, float Vy, float Vz, float theta)  //theta 机器人坐标系x轴与世界坐标系x轴夹角 单位 度
{
	theta = PI*theta/180.0f;
	MOTOR_STATE.Motor_A_SPEED   = cos(theta)*Vx         + sin(theta)*Vy        + L_PARAMETER*Vz;
	MOTOR_STATE.Motor_B_SPEED   = -cos(PI/3.0f-theta)*Vx + sin(PI/3.0f-theta)*Vy + L_PARAMETER*Vz;
	MOTOR_STATE.Motor_C_SPEED   = -cos(PI/3.0f+theta)*Vx - sin(PI/3.0f+theta)*Vy + L_PARAMETER*Vz;
}

/**************世界坐标系正运动学*****************/
//void World_Forward_Kinematic_Analysis(float theta)  //theta 机器人坐标系x轴与世界坐标系x轴夹角
//{
//	PVx = wheel_speed[0]*2*cos(theta)/3.0f - wheel_speed[1]*(cos(theta)+sqrt(3.0f)*sin(theta))/3.0f - wheel_speed[2]*(cos(theta)-sqrt(30)*sin(theta))/3.0f ;
//	PVy = wheel_speed[0]*2*sin(theta)/3.0f - wheel_speed[1]*(sin(theta)-sqrt(3.0f)*cos(theta))/3.0f - wheel_speed[2]*(sin(theta)+sqrt(30)*cos(theta))/3.0f ;
//	PVz = wheel_speed[0]/(3.0f*Radius)     + wheel_speed[1]/(3.0f*Radius)                           + wheel_speed[2]/(3.0f*Radius);
//}

int limit_acceleration(int last_speed, int now_speed, int max_accel)
{
	int accel;
	accel= abs(now_speed - last_speed);
	if(accel > max_accel)
	{
		if(now_speed - last_speed > 0)
		{
			now_speed = last_speed + max_accel;
		}
		else
		{
			now_speed = last_speed - max_accel;
		}
	}
	else
		return now_speed;
	
	return now_speed;
}

void motor_init()
{
	CAN1_Configuration();  //CAN1初始化   
	CAN2_Configuration();
	
//	#if motor_USEING_RE40
//	delay_ms(100);
//	CAN_RoboModule_DRV_Reset(0,0);                      //对0组1号驱动器进行复位
//	delay_ms(500);                                     //发送复位指令后的延时必须要有，等待驱动器复位完毕。
//	CAN_RoboModule_DRV_Config(0,0,1,0);               //配置为1ms传回一次数据
//	delay_ms(50);
//	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //选择进入速度模式
//	delay_ms(500);
//  #endif
//	
//	#if motor_USEING_M3508
//	PIDInit(&M3508_pid[0], 10.0 , 1.0 , 0.0 ,  15000.0, 15000.0);
//	PIDInit(&M3508_pid[1], 10.0 , 1.0 , 0.0 ,  15000.0, 15000.0);
//	PIDInit(&M3508_pid[2], 10.0 , 1.0 , 0.0 ,  15000.0, 15000.0);
//	#endif
}

void base_control_task()
{
	if(TRACKING_STATE != TRACKING_GOING || NOW_STATE == STOP_STATE)//
	{
		ROBOT_STATE.Vx = ROBOT_STATE.Vy = ROBOT_STATE.Vz = 0;
	}
	
	#if DEBUG_THROW_TASK
	ROBOT_STATE.Vx = ROBOT_STATE.Vy = ROBOT_STATE.Vz = 0;
	#endif
	
	/*********************最大速度限制***********************/
//	if(ROBOT_STATE.Vx>MAXIMUM_LINEAR_SPEED) ROBOT_STATE.Vx = MAXIMUM_LINEAR_SPEED;
//	else if(ROBOT_STATE.Vx<-MAXIMUM_LINEAR_SPEED) ROBOT_STATE.Vx = -MAXIMUM_LINEAR_SPEED;
//	
//	if(ROBOT_STATE.Vy>MAXIMUM_LINEAR_SPEED) ROBOT_STATE.Vy = MAXIMUM_LINEAR_SPEED;
//	else if(ROBOT_STATE.Vy<-MAXIMUM_LINEAR_SPEED) ROBOT_STATE.Vy = -MAXIMUM_LINEAR_SPEED;
//	
//	if(ROBOT_STATE.Vz>MAXIMUM_ANGULAR_SPEED) ROBOT_STATE.Vz = MAXIMUM_ANGULAR_SPEED;
//	else if(ROBOT_STATE.Vz<-MAXIMUM_ANGULAR_SPEED) ROBOT_STATE.Vz = -MAXIMUM_ANGULAR_SPEED;
	/*********************最大速度限制***********************/
	#if robot_coordinate
	Kinematic_Analysis(ROBOT_STATE.Vx, ROBOT_STATE.Vy, ROBOT_STATE.Vz);
	#else 
	World_Kinematic_Analysis(ROBOT_STATE.Vx, ROBOT_STATE.Vy, ROBOT_STATE.Vz, ROBOT_POSITION.ANGLE_POS);
	#endif
	

	ROBOT_STATE.Last_Vx = ROBOT_STATE.Vx;
	ROBOT_STATE.Last_Vy = ROBOT_STATE.Vy;
	ROBOT_STATE.Last_Vz = ROBOT_STATE.Vz;
	
//	#if motor_USEING_RE40
//  CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,MOTOR_STATE.Motor_A_SPEED);
//  CAN_RoboModule_DRV_Velocity_Mode(0,2,5000,MOTOR_STATE.Motor_B_SPEED);
//  CAN_RoboModule_DRV_Velocity_Mode(0,3,5000,MOTOR_STATE.Motor_C_SPEED);
//	#endif
	
	#if motor_USEING_M3508
	/**************电调默认方向与运动学设定正方向相反****************/
	MOTOR_STATE.Motor_A_SPEED = -MOTOR_STATE.Motor_A_SPEED;
	MOTOR_STATE.Motor_B_SPEED = -MOTOR_STATE.Motor_B_SPEED;
	MOTOR_STATE.Motor_C_SPEED = -MOTOR_STATE.Motor_C_SPEED;
	/***************电调默认方向与运动学设定正方向相反***************/
	
	/***************pid调参***************/
	#if DEBUG_M3508_PID_PARAM
	MOTOR_STATE.Motor_A_SPEED = m_target1;
	MOTOR_STATE.Motor_B_SPEED = m_target2;
	MOTOR_STATE.Motor_C_SPEED = m_target3;
	#endif
	/***************pid调参***************/
	
	PID_Incremental_Calc( &M3508_pid[0],  MOTOR[0].inner_rpm,  MOTOR_STATE.Motor_A_SPEED );
	PID_Incremental_Calc( &M3508_pid[1],  MOTOR[1].inner_rpm,  MOTOR_STATE.Motor_B_SPEED );
	PID_Incremental_Calc( &M3508_pid[2],  MOTOR[2].inner_rpm,  MOTOR_STATE.Motor_C_SPEED );
	//PID_Incremental_Calc( &M3508_pid[3],  MOTOR[3].inner_rpm,  MOTOR_STATE.Motor_D_SPEED );
	
	Set_Motor_Speed(CAN1, M3508_pid[0].output , M3508_pid[1].output, M3508_pid[2].output, M3508_pid[3].output);
	#endif
	
	
}

void locked_rotor_current_check()
{
	if(MOTOR[0].current>15000||MOTOR[1].current>15000||MOTOR[2].current>15000)
	{
		Set_Motor_Speed(CAN2, 0 , 0, 0, 0);
		return ;
	}
}


