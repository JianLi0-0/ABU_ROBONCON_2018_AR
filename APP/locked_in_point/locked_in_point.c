#include "main.h"
#define locked_angle_threshold 0.1f
#define locked_pos_threshold 1.0f

PID pid_locked_in_point;
PID pid_locked_in_angle;
GOAL_ERRORS locked_error;

POINT POINT_STZ = {-565.0,4565.0,0,1};
POINT POINT_RELAY1 = {-565.0,4565.0,0,1};
POINT POINT_TZ1 = {-3260.00000,4565.0,0,1};
POINT POINT_TZ2 = {-3260.00000000,6565.00000000,0,1};	
POINT POINT_RELAY2 = {-565.000000000,6565.0,0,1};	
POINT POINT_TZ3 = {-6520.00000000,6565.0,0,1};
POINT NOW_LOCKED_POINT;

locking_STATE LOCKING_STATE = LOCKING_UNKNOWN;

void LOCKED_IN_POINT_TASK_INIT()
{
	PIDInit(&pid_locked_in_point, 50.0, 0.0, 1.0, 1000.0, 1000.0); 
	PIDInit(&pid_locked_in_angle, 50.0, 0.0, 1.0, 1000.0, 1000.0);
}

void LOCKED_IN_POINT_TASK()
{
	switch(NOW_STATE)
	{
		case STZ_TO_RELAY1 : 
		{
			NOW_LOCKED_POINT = POINT_RELAY1;
		}
		break;

		case RELAY1_TO_TZ1 : 
		{
			NOW_LOCKED_POINT = POINT_TZ1;
		}
		break;

		case TZ1_TO_RELAY1 : 
		{
			NOW_LOCKED_POINT = POINT_RELAY1;
		}
		break;

		case RELAY1_TO_TZ2 : 
		{
			NOW_LOCKED_POINT = POINT_TZ2;
		}
		break;

		case TZ2_TO_RELAY2 : 
		{
			NOW_LOCKED_POINT = POINT_RELAY2;
		}
		break;

		case RELAY2_TO_TZ3 : 
		{
			NOW_LOCKED_POINT = POINT_TZ3;
		}
		break;
		
		case RELAY2_TO_TZ2 :
		{
			NOW_LOCKED_POINT = POINT_TZ2;
		}
		
		case TZ3_TO_RELAY2 : 
		{
			NOW_LOCKED_POINT = POINT_RELAY2;
		}
		break;
		default:{}
	}
	
	if(TRACKING_STATE == TRACKING_ARRIVED)
	{
		LOCKED_IN_POINT(NOW_LOCKED_POINT);
	}
	
}

void LOCKED_IN_POINT(POINT NOW_L_POINT)
{
	locked_error.x_error = NOW_L_POINT.x - ROBOT_POSITION.X_POS;
	locked_error.y_error = NOW_L_POINT.y - ROBOT_POSITION.Y_POS;
	locked_error.distance_error = sqrt(locked_error.x_error*locked_error.x_error + locked_error.y_error*locked_error.y_error);
	locked_error.z_error = NOW_L_POINT.z - ROBOT_POSITION.ANGLE_POS;
	
	PID_Position_Calc(&pid_locked_in_angle, ROBOT_POSITION.ANGLE_POS, NOW_L_POINT.z);  //位置式pid
	ROBOT_STATE.Vz = pid_locked_in_angle.output;

	PID_Position_Calc(&pid_locked_in_point, locked_error.distance_error, 0.0);
	ROBOT_STATE.Vx = pid_locked_in_point.output * locked_error.x_error / locked_error.distance_error;  //tracking_speed * locked_error.x_error / locked_error.xy_error;  //
	ROBOT_STATE.Vy = pid_locked_in_point.output * locked_error.y_error / locked_error.distance_error;  //tracking_speed * locked_error.y_error / locked_error.xy_error;  
	
	if(  ABSOLUTE(locked_error.z_error) < locked_angle_threshold && ABSOLUTE(locked_error.distance_error) < locked_pos_threshold)  //5-1更改过
	{
		ROBOT_STATE.Vx = 0;
		ROBOT_STATE.Vy = 0;
    ROBOT_STATE.Vz = 0;
		LOCKING_STATE = LOCKING_SUCCEEDED;
	}

}
