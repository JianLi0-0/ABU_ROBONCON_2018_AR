#ifndef __MAIN_H
#define __MAIN_H

#include "can1.h"
#include "can2.h"
#include "timer2.h"
#include "delay.h"
#include "usart.h"
#include "base_control.h"
#include "vison_uart.h"
#include "sys.h"
#include "delay.h"
#include "timer2.h"
#include "task.h"
#include "point_to_point.h"
#include "PID.h"
#include "throw.h"
#include "math.h"
#include "led.h"
#include "global_localization.h"
#include "scope.h"
#include "relay.h"
#include "remoter.h"
#include "pstwo.h"
#include "Supervise.h"
#include "stdint.h"
#include "gyro_uart.h"
#include "encoder.h"
#include "exti.h"
#include "LIDAR_usart.h"
#include "iomanagement.h"
#include "locked_in_point.h"
#include "kalman_2dfilter.h"
#include "lcd.h"
#include "usart2.h"	 
#include "beep.h" 
#include "key.h"
#include "Servo.h"
#include "exception_handling.h"
#include "xunxian.h"
#define ABSOLUTE(x) ((x)>0? (x):(-(x)))
#define PI                   (3.14159265f)

#define hong1 1
#define hong2 0
#define lan1 0
#define lan2 0

#define DEBUG_REMOTER 0

#define DEBUG_AUTOROBOT_NOESELF    0 //
#define DEBUG_ROBOT_BASE  0  //throw state autochange,not need motor
#define DEBUG_THROW_TASK  0


/*************global_localization.c************/
#define DEBUG_ENCODER 0  //t3 t4 t8

#define ENCODER_ONLY 0
//#define USING_Grayscale_Sensor 0

#define LIDAR_ONLY 0
#define ENCODER_LIDAR_FUSION 1
/*************global_localization.c************/

/*************PRINTF************/
#define DEBUG_THROW_MOTOR 0  //print motor speed
#define DEBUG_HARDWARE 0
#define DEBUG_POS_AND_SPD 0   //print pos and spd
#define DEBUG_LIDAR_POS 0
/*************task.c************/


/*************base_control.c************************/
#define DEBUG_M3508_PID_PARAM 0   //pid调参
#define DEBUG_SERVO 0  //pid调参
/*************base_control.c************************/

/*************servo.c*************************************/
#define LEFT_SERVOR_INIT     800
#define LEFT_SERVOR_LOCKED  3700
#define LEFT_TZ1_LOCKED   3700
#define LEFT_TZ2_LOCKED   4000
#define LEFT_TZ3_LOCKED   3800

#define RIGHT_SERVOR_INIT    1800
#define RIGHT_SERVOR_LOCKED  1400
#define RIGHT_TZ1_LOCKED  1400
#define RIGHT_TZ2_INIT    1800
#define RIGHT_TZ2_LOCKED  1400
#define RIGHT_TZ3_LOCKED  1300

#define LEFT_CLAMP_INIT     1600
#define LEFT_CLAMP_LOCKED   4600
#define RIGHT_CLAMP_INIT    3500
#define RIGHT_CLAMP_LOCKED  1400

/*************servo.c*************************************/

/*************delay_ms*************************************/

#define delay_before_throw 300

/*************delay_ms*************************************/

extern R_S ROBOT_STATE;
extern M_S MOTOR_STATE;    

extern PID M3508_pid[4];
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float w_z;
extern Motor MOTOR[4];

extern POINT point[];
extern POINT inverse_point[];
extern POINT STARTZONE_TO_RELAY1[];	
extern POINT RELAY1_TO_THROWZONE1[];	
extern POINT THROWZONE1_TO_RELAY1[];	
extern POINT RELAY1_TO_THROWZONE2[];	
extern POINT THROWZONE2_TO_RELAY2[];	
extern POINT RELAY2_TO_THROWZONE3[];
extern POINT RELAY2_TO_THROWZONE2[];
extern POINT THROWZONE3_TO_RELAY2[];

extern POINT STARTZONE_TO_RELAY2[];


extern uint32_t missing_counter[MISSING_COUNTER_NUM];

extern ROBOT_WORKING_STATE LAST_STATE;
extern ROBOT_WORKING_STATE NOW_STATE;
extern Throw_STATE THROW_STATE;
extern Throw_STAGE THROW_STAGE;
extern Tracking_STATE LAST_TRACKING_STATE;
extern Tracking_STATE TRACKING_STATE;
extern Relay_STATE RELAY_STATE;
extern Relay_STATE RELAY_STATE;
extern locking_STATE LOCKING_STATE;

extern state_flag STATE_flag;
extern char VISION_MES;
extern u8 init ;
extern u8 ZONE_TO_GO;

extern Enco Encoder;
extern ENCO_POS ROBOT_ENCO_POS;
extern ROBOT_POS ROBOT_POSITION;
extern ROBOT_POS LIDAR_POSITION;
extern lidar LIDAR_POS;

extern color COLOR;
extern uint32_t lost_err;

extern u8 IS_LIDAR_RECEIVED;
#endif
