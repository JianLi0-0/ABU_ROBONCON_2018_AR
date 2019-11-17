#ifndef __THROW_H
#define __THROW_H


void THROW_TASK(void);
void THROW_TASK_INIT(void);
void throw_ball(void);
int limit_acceleration(int last_speed, int now_speed, int max_accel);
void return_start_pos(void);
void DOCKING(void);
typedef struct throwing_param {
  float  start_angle;
	float  accelerate_angle;
  float  break_angle;
  float  stop_angle;
	float  start_position;
	float  accelerate_position;
  float  break_position;
  float  stop_position;
	int throw_speed;
	int speed_after_decelerate;
	int ACCELERATE;
	int DECELERATE;
}throwing_param;


typedef struct TM_S{
	unsigned char LEFT_NUM;
  int Motor_1eft_SPEED;
	int LAST_Motor_1eft_SPEED;
	long Motor_1eft_pos;
	
	unsigned char RIGHT_NUM;
  int Motor_right_SPEED;
	int Motor_right_ACCELERATE;
	int Motor_right_DECELERATE;
	int LAST_Motor_right_SPEED;
	long Motor_right_pos;
}TM_S;
typedef struct Throwing_M{
	unsigned char NUM;
  int Motor_SPEED;
	int LAST_Motor_SPEED;
	long Motor_pos;
}Throwing_M;



#endif
