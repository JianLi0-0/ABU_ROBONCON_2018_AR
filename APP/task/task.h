#ifndef __TASK_H
#define __TASK_H
#include "sys.h"

extern uint8_t Bsp_Int_Ok; 


void Nvic_Init(void); 
void BSP_Init(void);
void Task_1000HZ(void);
void Task_500HZ(void);
void Task_250HZ(void);
void Task_200HZ(void);
void Task_100HZ(void);
void Task_50HZ(void);
void Task_20HZ(void);
void Task_10HZ(void);
void Task_5HZ(void);
void Task_1HZ(void);

typedef enum
{
	PREPARE_STATE,
	
	STZ_TO_RELAY1,
	RELAY1_TO_TZ1,  //2
	TZ1_TO_RELAY1,
	RELAY1_TO_TZ2,  //4
	TZ2_TO_RELAY2,
	RELAY2_TO_TZ3,  //6
	RELAY2_TO_TZ2,
	TZ3_TO_RELAY2,
	
	THROW_ZONE1,
	THROW_ZONE2,
	THROW_ZONE3,
	
	RELAY_STATE1,  //RELAY1 TO TZ1
	RELAY_STATE2,  //RELAY1 TO TZ2
	RELAY_STATE3,  //RELAY2 TO TZ2
	RELAY_STATE4,  //RELAY2 TO TZ3
	
	STOP_STATE,
	
	STZ_TO_RELAY2,
	
}ROBOT_WORKING_STATE;


typedef enum
{
  THROW_SUCCEEDED,
	THROW_FAILED,
	THROW_UNKNOWN,
	THROW_STATE_WAITING,
}Throw_STATE;  //THROW_STATE

typedef enum
{
	THROWING,
  THROW_ENDED,
	THROW_NOTKNOW,

}Throw_STAGE;  //THROW_STAGE

typedef enum
{
  TRACKING_ARRIVED,
	TRACKING_GOING,
	TRACKING_UNKNOWN,
	
}Tracking_STATE;  //TRACKING_STATE

typedef enum
{
  RELAY_SUCCEEDED,
	RELAY_WAITING,
	RELAY_UNKNOWN,
	RELAY_GOING,
}Relay_STATE;  //RELAY_STATE

typedef enum
{
  DOCKING_SUCCEEDED,
	DOCKING_UNKNOWN,
	DOCKING_GOING,
}Docking_STATE;  //RELAY_STATE

typedef enum
{
  LOCKING_SUCCEEDED,
	LOCKING_UNKNOWN,
	LOCKING_GOING,
}locking_STATE;  //RELAY_STATE


ROBOT_WORKING_STATE get_robotstate(void);
void Finite_state_machine(void);
void set_robotstate(ROBOT_WORKING_STATE state);
void Artifiial_delay(int delay_time,int period);

#endif
