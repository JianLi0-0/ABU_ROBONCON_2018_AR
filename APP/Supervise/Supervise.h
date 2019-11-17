#ifndef __SUPERVISE_H
#define __SUPERVISE_H
#include "stdint.h"


#define MISSING_COUNTER_NUM              11u

#define MISSING_COUNTER_INDEX_GYRO  				0u  //gyro : 1st led 4/1s toggle; 0001

#define MISSING_COUNTER_INDEX_MOTOR1        1u  //five motor : 1st led 2/1s toggle; 001 010 011 100 101 always on 1~5motor
#define MISSING_COUNTER_INDEX_MOTOR2        2u
#define MISSING_COUNTER_INDEX_MOTOR3        3u

#define MISSING_COUNTER_INDEX_MOTOR4        4u
#define MISSING_COUNTER_INDEX_MOTOR5        5u
#define MISSING_COUNTER_INDEX_LIDAR         6u  //lidar : 1st led 4/1s toggle; 0010
//#define MISSING_COUNTER_INDEX_MOTOR6        7u

#define MISSING_COUNTER_INDEX_INFRARED_SENS 8u
//#define MISSING_COUNTER_INDEX_LIDAR         9u  
#define MISSING_COUNTER_INDEX_UNKNOWN       10u

#define MISSING_ERROR_GYRO									(1<<0)		//
#define MISSING_ERROR_MOTOR1								(1<<1)		//
#define MISSING_ERROR_MOTOR2								(1<<2)		//
#define MISSING_ERROR_MOTOR3								(1<<3)		//
#define MISSING_ERROR_MOTOR4								(1<<4)		//
#define MISSING_ERROR_MOTOR5								(1<<5)		//
//#define MISSING_ERROR_MOTOR6								(1<<6)		//
#define MISSING_ERROR_LIDAR							(1<<6)		//
#define MISSING_ERROR_MO							(1<<8)		//
#define MISSING_ERROR_DEADLOCK								(1<<9)		//deadlock error
#define MISSING_ERROR_NOCALI  						        (1<<10)		//nocali error


void Supervise(void);
void LCD_TASK(void);
void LCD_PREPARE(void);
void LCD_PAGE1_SENSORS(void);
void LCD_PAGE3_TRACKING(void);
void LCD_PAGE2_ROBOTSTATE(void);
uint32_t Get_Lost_Error(uint32_t err_code);
//uint32_t *GetMissingCounter(uint8_t index);
void MissingCounterCount(uint32_t *mc, uint32_t os_ticks);
uint8_t MissingCounterOverflowCheck(uint32_t mc, uint32_t threshold);
void LostCounterFeed(uint32_t *mc);


void senddata(float a);
void PAGE0_TXT0(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten,float ele);
void PAGE0_TXT1(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten);
void PAGE1_TXT0(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten);
									
#endif
