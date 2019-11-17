#include "PID.h"
#include "math.h"
float abs_limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}

void PID_Position_Calc( PID *pp,  float  CurrentPoint,  float NextPoint )  
{   
        pp->Error =  NextPoint -  CurrentPoint;          
        pp->SumError += pp->Error;                      
        pp->DError = pp->Error - pp->LastError;
	
        pp->output =  pp->Proportion * pp->Error +   \
											abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
											pp->Derivative * pp->DError ;  

	      if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	      if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax;
//	      pp->PrevError = pp->LastError;  
        pp->LastError = pp->Error;
}

void PID_Incremental_Calc( PID *pp,  float  CurrentPoint,  float NextPoint )  
{  
        pp->Error =  NextPoint -  CurrentPoint;          
        pp->SumError += pp->Error;                      
        pp->DError = pp->Error - pp->LastError;
	
        pp->output +=  pp->Proportion * ( pp->Error - pp->LastError )+   \
											 abs_limit(pp->Integral * pp->Error, pp->Integralmax ) +   \
											 pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);  
	
	      if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	      if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax;
	      pp->PrevError = pp->LastError;  
        pp->LastError = pp->Error;
}

void PIDInit(PID *pp, float Kp , float Ki , float Kd ,  float outputmax, float Integralmax)  
{  
	  pp->Integralmax = pp->outputmax  = outputmax;
	  pp->Proportion = Kp;
	  pp->Integral   = Ki;
	  pp->Derivative = Kd;
    pp->DError = pp->Error = pp->output = pp->LastError = pp->PrevError = 0; 
}  

void Fuzzy_PID_Calc(FUZZY_PID *fp,  float  CurrentPoint,  float NextPoint ) 
{
	pid_factor_type pid_factor;  /* pidϵ�� */
	float max_output;
	
	fp->LastError = fp->Error;
	fp->Error =  NextPoint -  CurrentPoint;          
	fp->SumError += fp->Error;                      
	fp->DError = fp->Error - fp->LastError;
	
	if (fabsf(fp->Error) >= fp->Error_Threshold_Big)
	{
		pid_factor = fp->error_big_pid; /* ʹ�ô�����Ӧ��pid���� */
	}
	
	/* ����е� */
	else if (fabsf(fp->Error) >= fp->Error_Threshold_Small)
	{
		pid_factor = fp->error_medium_pid; /* ʹ���е�����Ӧ��pid���� */
	}
	
	/* ����С */
	else if (fabsf(fp->Error) >= fp->Error_Threshold_Deadzone)
	{
		pid_factor = fp->error_small_pid; /* ʹ��С����Ӧ��pid���� */
	}
	
	/* ����������� */
	else
	{
		pid_factor.p = pid_factor.i = pid_factor.d = 0; /* ����ֹͣ��� */
	}
	
	
	
		/* ���������Ų�ͬ, ��������� */
	if ((fp->LastError > 0 && fp->Error < 0)
		|| (fp->LastError < 0 && fp->Error > 0))
	{
		fp->SumError = 0;
	}
	
		/* ������ = ���λ���(��ʱ�����, ���ܿ�����������) */
	fp->SumError += fp->Control_Period 
				* (fp->Error + fp->LastError) / 2; 
	/* ���΢�� = ����ֵ / �������� */
	fp->DError = (fp->Error - fp->LastError) 
				/ fp->Control_Period;
	/* ��¼������ֵ */
	if (fabsf(fp->Error) > fp->Max_Error)
	{
		fp->Max_Error = fabsf(fp->Error);
	}
	
		fp->output = pid_factor.p * fp->Error
				+ pid_factor.i * fp->SumError
				+ pid_factor.d * fp->DError;
	/* pid�������ѡ��: �ⲿĳ����(����ָ��ʹ��) �� ָ����ĳ���� */
//	if (pid_struct->output_limit_pointer_abs != NULL
//		&& fabsf(*pid_struct->output_limit_pointer_abs) > fabsf(pid_struct->output_limit_abs))
//	{
//		max_output = fabsf(*pid_struct->output_limit_pointer_abs);
//	}
//	/* �������ָ���ı��� */
//	else 
//	{
//		max_output = fabsf(pid_struct->output_limit_abs);
//	}
	max_output = fabsf(fp->outputmax);
	/* �������, ��С����, ���Ų��� */
	if (fabsf(fp->output) > fabsf(max_output))
	{
		fp->output *= fabsf(max_output) / fabsf(fp->output);  
	}
	
}
