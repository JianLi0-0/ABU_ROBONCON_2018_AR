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



/* pid系数 */
typedef struct
{
	float p; /* 比例系数 */
	float i; /* 积分系数 */
	float d; /* 微分系数 */
}pid_factor_type;

/*模糊pid*/
typedef struct FUZZY_PID {
	pid_factor_type error_big_pid; /* 误差大/中/小三个区间的不同系数 */
	pid_factor_type error_medium_pid;
	pid_factor_type error_small_pid;
	
	float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
	float  Error;
	float  DError;
	float  Max_Error;
  float  SumError;           //  Sums of Errors
	float  Integralmax;

	float  Control_Period;     //控制周期
	
	float  Error_Threshold_Big; //大偏差阈值
	float  Error_Threshold_Small;
	float  Error_Threshold_Deadzone;

	float  output;
	float  outputmax;
}FUZZY_PID;


/*----------------------  初 始 化 例 子  -------------------------*/

///* pid */
//pid_control_type cm_speed_pid =
//{
//	.error_big_pid    = {.p = 0.8f, .i = 0, .d = 0}, /* 误差大/中/小三个区间的不同系数 */
//	.error_threshold_big   = 20, /* 大误差阈值 */
//	.error_medium_pid = {.p = 0.4f, .i = 0, .d = 0},
//	.error_threshold_small = 10, /* 小误差阈值 */
//	.error_small_pid  = {.p = 0.1f, .i = 0, .d = 0},
//	.error_dead_space = 0,

//	.control_cycle         = 1, /* 控制周期, 与微分和积分有关 */

//	/* 每次限制取以下两个中最大的一个 */
//	.output_limit_abs         = 500, /* 输出限制 */
//	.output_limit_pointer_abs = NULL, /* 输出限制指针(被指向对象的值即为限制值) */
//};


float abs_limit(float a, float ABS_MAX);
void PID_Position_Calc( PID *pp,  float  CurrentPoint,  float NextPoint);
void PID_Incremental_Calc( PID *pp,  float  CurrentPoint,  float NextPoint);
void PIDInit(PID *pp, float Kp , float Ki , float Kd ,  float outputmax, float Integralmax);
void Fuzzy_PID_Calc(FUZZY_PID *fp,  float  CurrentPoint,  float NextPoint ) ;
#endif
