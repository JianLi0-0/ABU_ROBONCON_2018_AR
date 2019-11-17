#ifndef __PID_H
#define __PID_H
typedef struct PID {
  float  Proportion;         //  ???? Proportional Const  
  float  Integral;           //  ???? Integral Const  
  float  Derivative;         //  ???? Derivative Const  
	float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
	float  Error;
	float  DError;
  float  SumError;           //  Sums of Errors  
	float  Integralmax;
	float  output;
	float  outputmax;
} PID;



/* pidϵ�� */
typedef struct
{
	float p; /* ����ϵ�� */
	float i; /* ����ϵ�� */
	float d; /* ΢��ϵ�� */
}pid_factor_type;

/*ģ��pid*/
typedef struct FUZZY_PID {
	pid_factor_type error_big_pid; /* ����/��/С��������Ĳ�ͬϵ�� */
	pid_factor_type error_medium_pid;
	pid_factor_type error_small_pid;
	
	float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
	float  Error;
	float  DError;
	float  Max_Error;
  float  SumError;           //  Sums of Errors
	float  Integralmax;

	float  Control_Period;     //��������
	
	float  Error_Threshold_Big; //��ƫ����ֵ
	float  Error_Threshold_Small;
	float  Error_Threshold_Deadzone;

	float  output;
	float  outputmax;
}FUZZY_PID;


/*----------------------  �� ʼ �� �� ��  -------------------------*/

///* pid */
//pid_control_type cm_speed_pid =
//{
//	.error_big_pid    = {.p = 0.8f, .i = 0, .d = 0}, /* ����/��/С��������Ĳ�ͬϵ�� */
//	.error_threshold_big   = 20, /* �������ֵ */
//	.error_medium_pid = {.p = 0.4f, .i = 0, .d = 0},
//	.error_threshold_small = 10, /* С�����ֵ */
//	.error_small_pid  = {.p = 0.1f, .i = 0, .d = 0},
//	.error_dead_space = 0,

//	.control_cycle         = 1, /* ��������, ��΢�ֺͻ����й� */

//	/* ÿ������ȡ��������������һ�� */
//	.output_limit_abs         = 500, /* ������� */
//	.output_limit_pointer_abs = NULL, /* �������ָ��(��ָ������ֵ��Ϊ����ֵ) */
//};


float abs_limit(float a, float ABS_MAX);
void PID_Position_Calc( PID *pp,  float  CurrentPoint,  float NextPoint);
void PID_Incremental_Calc( PID *pp,  float  CurrentPoint,  float NextPoint);
void PIDInit(PID *pp, float Kp , float Ki , float Kd ,  float outputmax, float Integralmax);
void Fuzzy_PID_Calc(FUZZY_PID *fp,  float  CurrentPoint,  float NextPoint ) ;
#endif
