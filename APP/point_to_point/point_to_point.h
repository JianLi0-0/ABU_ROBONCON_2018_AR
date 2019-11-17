#ifndef __POINT_TO_POINT_H
#define __POINT_TO_POINT_H

typedef struct POINT{
    float x;
    float y;
    float z;
		char stop;
}POINT;

typedef struct GOAL_ERRORS{
    float x_error;
    float y_error;
    float xy_error;
	  float distance_error;
		float z_error;
	  float distance_between_2pts;
	  float ran_distance;
	  float running_speed;
}GOAL_ERRORS;

typedef struct trapezoid_param{
	float trapezoid_maximum;
	float trapezoid_minimum;
	float accel_coefficient;
	float decel_coefficient;
}trapezoid_param;

extern GOAL_ERRORS goal_errors;
extern int count;

void TRACKING_TASK(POINT *POINTS);
void track_point(POINT *POINTS);
void tracking_init(void);
#endif
