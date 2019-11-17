#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"

void CAN2_Configuration(void);

#define RATE_BUF_SIZE 6

/*************************************************************************
                          M3508
*************************************************************************/
#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID           0x204


typedef struct{
	
	uint16_t angle;   									//编码器不经处理的原始值
	int16_t inner_rpm;								//上一次的编码器原始值
	int16_t current;                       //经过处理后连续的编码器值
//	int32_t diff;													//两次编码器之间的差值
//	int32_t temp_count;                   //计数用
//	uint8_t buf_count;								//滤波更新buf用
//	int32_t ecd_bias;											//初始编码器值	
//	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
//	int32_t round_cnt;										//圈数
//	int32_t filter_rate;											//速度
//	float ecd_angle;											//角度
//	union
//	{
//	 uint8_t data[8];
//	 uint16_t ActVal[4];
//	}fdb;
}Motor;

void Set_Motor_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void get_m3508_fdb(CanRxMsg * msg);

//typedef struct{
//	int32_t raw_value;   									//编码器不经处理的原始值
//	int32_t last_raw_value;								//上一次的编码器原始值
//	int32_t ecd_value;                       //经过处理后连续的编码器值
//	int32_t diff;													//两次编码器之间的差值
//	int32_t temp_count;                   //计数用
//	uint8_t buf_count;								//滤波更新buf用
//	int32_t ecd_bias;											//初始编码器值	
//	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
//	int32_t round_cnt;										//圈数
//	int32_t filter_rate;											//速度
//	float ecd_angle;											//角度
//}Encoder;
/*************************************************************************
                          M3508
*************************************************************************/
#endif 
