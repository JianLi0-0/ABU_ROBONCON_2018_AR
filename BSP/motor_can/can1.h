#ifndef __CAN_H__
#define __CAN_H__

#include "stdint.h"
#include "stm32f4xx.h"

#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08



#define MAIN_CONTROLLER_XY_MSG_ID        0x7AF 
#define MAIN_CONTROLLER_ANGLE_MSG_ID     0x7BF
#define MAIN_CONTROLLER_OTHER_MSG_ID     0x7CF

#define LIDAR_XY_MSG_ID                  0x7FF 
#define LIDAR_ANGLE_MSG_ID               0x7EF
#define LIDAR_ERROR_MSG_ID               0x7DF

/*************************************************************************
                          M3508
*************************************************************************/

void CAN1_Configuration(void);

void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number);



void GET_LIDAR_MSG(CanRxMsg * msg);
void SEND_LIDAR_XY_MSG(CAN_TypeDef *CANx, float pos_x, float pos_y);
void SEND_LIDAR_ANGLE_MSG(CAN_TypeDef *CANx, float pos_angle);
void SEND_LIDAR_ERROR_MSG(CAN_TypeDef *CANx, u8 start);

extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];
extern char ERROR_MSG[8];


void Odrive_Control(CAN_TypeDef *CANx, u8 motor_num, int set_point, u8 control_mode);
void Get_Encoder_Estimates_Tx(CAN_TypeDef *CANx, u8 drive_num);
void Get_Encoder_Estimates_Rx(CanRxMsg * msg);

#endif
