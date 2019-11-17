#include "fuzzy_pid.h"
#include "math.h" /* ???? */


/*----------------------  ? ? ? ? ?  -------------------------*/

///* ????pid */
//pid_control_type path_correct_pid =
//{
//	.error_big_pid    = {.p = 0.15f, .i = 0, .d = 0}, /* ???/?/?????????? */
//	.error_medium_pid = {.p = 0.12f, .i = 0, .d = 0},
//	.error_small_pid  = {.p = 0.08f, .i = 0, .d = 0},

//	.control_cycle         = GPS_UPDATE_CYCLE, /* ????, ???????? */
//	.error_threshold_big   = 50, /* ????? */
//	.error_threshold_small = 10, /* ????? */
//	.error_dead_space 	   = 0, /* ???? */

//	/* ??????????????? */
//	.output_limit_abs         = 15, /* ???? */
//	.output_limit_pointer_abs = &speed.actual_speed, /* ??????(????????????) */
//};

/*-------------------------------------------------------------------*/


/*
* ??: pid_control
* ??: pid??, ????????(?????pid??)
* ??: pid_struct, ???????pid
*		error ???
*/
float pid_control(pid_control_type *pid_struct, float error)
{
	float output_value; /* ??? */
	float max_output; /* ?????? */
	pid_factor_type pid_factor; /* pid?? */
	
	
	/*-----------------------------------------------------        pid ? ? ? ?        ----------------------------------------------------*/
	/* ???? */
	if (fabsf(error) >= pid_struct->error_threshold_big)
	{
		pid_factor = pid_struct->error_big_pid; /* ????????pid?? */
	}
	
	/* ???? */
	else if (fabsf(error) >= pid_struct->error_threshold_small && fabsf(error) < pid_struct->error_threshold_big)
	{
		pid_factor = pid_struct->error_medium_pid; /* ?????????pid?? */
	}
	
	/* ???? */
	else if (fabsf(error) >= pid_struct->error_dead_space && fabsf(error) < pid_struct->error_threshold_small)
	{
		pid_factor = pid_struct->error_small_pid; /* ????????pid?? */
	}
	
	/* ?????? */
	else
	{
		pid_factor.p = pid_factor.i = pid_factor.d = 0; /* ?????? */
	}
	
	/*----------------------------------------------------          ? ? ? ?         -----------------------------------------------*/
	pid_struct->previous_error = pid_struct->current_error; /* ?????? */
	pid_struct->current_error = error; /* ?????? */
	
	/* ????????, ?????? */
	if ((pid_struct->previous_error > 0 && pid_struct->current_error < 0)
		|| (pid_struct->previous_error < 0 && pid_struct->current_error > 0))
	{
		pid_struct->integrate_error = 0;
	}
	
	/* ???? = ????(?????, ????????) */
	pid_struct->integrate_error += pid_struct->control_cycle 
				* (pid_struct->current_error + pid_struct->previous_error) / 2; 
	/* ???? = ???? / ???? */
	pid_struct->differential_error = (pid_struct->current_error - pid_struct->previous_error) 
				/ pid_struct->control_cycle;
	/* ??????? */
	if (fabsf(pid_struct->current_error) > pid_struct->max_error)
	{
		pid_struct->max_error = fabsf(pid_struct->current_error);
	}
	
	/*---------------------------------------------------    ? ? ? ? ? ? ?     -------------------------------------------------*/
	output_value = pid_factor.p * pid_struct->current_error
				+ pid_factor.i * pid_struct->integrate_error
				+ pid_factor.d * pid_struct->differential_error;
	/* pid??????: ?????(??????) ? ?????? */
//	if (pid_struct->output_limit_pointer_abs != NULL
//		&& fabsf(*pid_struct->output_limit_pointer_abs) > fabsf(pid_struct->output_limit_abs))
//	{
//		max_output = fabsf(*pid_struct->output_limit_pointer_abs);
//	}
//	/* ????????? */
//	else 
//	{
//		max_output = fabsf(pid_struct->output_limit_abs);
//	}
	max_output = fabsf(pid_struct->output_limit_abs);
	/* ????, ????, ???? */
	if (fabsf(output_value) > fabsf(max_output))
	{
		output_value *= fabsf(max_output) / fabsf(output_value);  
	}
	
	return output_value;
}

