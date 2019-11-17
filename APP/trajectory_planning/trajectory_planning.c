#include "trajectory_planning.h"
#include "main.h"
/*reduction ratio:19.2032085  r_wheel(r/min) = (v/(152*pi)) *60    r_motor = r_wheel *reduction ratio = v* (1/(152*pi)) *60 * 19.2032085 = v * 2.412857   */
#define MILIMETER_PER_SEC_TO_RPM 2.412857f   // v: mm/s..����ֱ��152mm��������ٱ�3591:187. ����ת��: n/min,n/60s,����һȦpi*152mm=  ;
/*Ԥ�����3m/s*/

int TRA_CNT;
PID PID_TRA_YAW,PID_SPD_X,PID_SPD_Y,PID_POS_X,PID_POS_Y;
int TRA_points;
compensation COMPENSATION;
void TARJECYTORY_PLANNING_INIT()
{
	PIDInit(&PID_TRA_YAW, 1.0, 0.0, 0.0, 1000.0, 1000.0);
	PIDInit(&PID_SPD_X, 1.0, 0.0, 0.0, 1000.0, 1000.0);
	PIDInit(&PID_SPD_Y, 1.0, 0.0, 0.0, 1000.0, 1000.0);
	PIDInit(&PID_POS_X, 1.0, 0.0, 0.0, 1000.0, 1000.0);
	PIDInit(&PID_POS_Y, 1.0, 0.0, 0.0, 1000.0, 1000.0);
}
void REAL_STATE_Cal(float delta_t)
{
	ROBOT_STATE.REAL_Vx = (ROBOT_POSITION.X_POS - ROBOT_POSITION.LAST_X_POS)/delta_t;
	ROBOT_STATE.REAL_Vy = (ROBOT_POSITION.Y_POS - ROBOT_POSITION.LAST_Y_POS)/delta_t;
	ROBOT_STATE.REAL_Vz = (ROBOT_POSITION.ANGLE_POS - ROBOT_POSITION.LAST_ANGLE_POS)/delta_t;
}

void TARJECYTORY_PLANNING_TASK()
{
	/******************100hzִ��*******************/
	TRA_CNT++;
	TRA_points = (int)(sizeof(PALNNING1)/sizeof(PALNNING1[0]));
	
	/*********************��ȡ�滮�ٶ�*********************/
	ROBOT_STATE.TARGET_Vx = MILIMETER_PER_SEC_TO_RPM * PALNNING1[TRA_CNT].Vx;
	ROBOT_STATE.TARGET_Vy = MILIMETER_PER_SEC_TO_RPM * PALNNING1[TRA_CNT].Vy;
	ROBOT_STATE.Vx = ROBOT_STATE.TARGET_Vx;
	ROBOT_STATE.Vy = ROBOT_STATE.TARGET_Vy;
	
	/**********************������ʵ�ٶ�**********************/
	REAL_STATE_Cal(0.01);
	
	/**********************��̬�ջ�**********************/
	PID_Position_Calc(&PID_TRA_YAW, ROBOT_POSITION.ANGLE_POS, PALNNING1[TRA_CNT].Angle);  //λ��ʽpid
	ROBOT_STATE.Vz = PID_TRA_YAW.output;
}

void SPEED_COMPENSATION()
{
	/**********************1khzִ�У��ٶȲ���*********************/
	/**********************PID����**********************/
	PID_Position_Calc(&PID_SPD_X, ROBOT_STATE.REAL_Vx, ROBOT_STATE.TARGET_Vx);
	PID_Position_Calc(&PID_SPD_Y, ROBOT_STATE.REAL_Vy, ROBOT_STATE.TARGET_Vy);
	COMPENSATION.SPD_COMPENS_X =  PID_SPD_X.output;
	COMPENSATION.SPD_COMPENS_Y =  PID_SPD_Y.output;
}

void POSITION_COMPENSATION()
{
	/**********************1khzִ�У�λ�ò��������켣****************/
	/**********************PID����***********************************/
	PID_Position_Calc(&PID_POS_X, 0, 0);  //xƫ�����
	PID_Position_Calc(&PID_POS_Y, 0, 0);  //yƫ�����
	COMPENSATION.POS_COMPENS_X = PID_POS_X.output;
	COMPENSATION.POS_COMPENS_Y = PID_POS_Y.output;
}
