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
	
	uint16_t angle;   									//���������������ԭʼֵ
	int16_t inner_rpm;								//��һ�εı�����ԭʼֵ
	int16_t current;                       //��������������ı�����ֵ
//	int32_t diff;													//���α�����֮��Ĳ�ֵ
//	int32_t temp_count;                   //������
//	uint8_t buf_count;								//�˲�����buf��
//	int32_t ecd_bias;											//��ʼ������ֵ	
//	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
//	int32_t round_cnt;										//Ȧ��
//	int32_t filter_rate;											//�ٶ�
//	float ecd_angle;											//�Ƕ�
//	union
//	{
//	 uint8_t data[8];
//	 uint16_t ActVal[4];
//	}fdb;
}Motor;

void Set_Motor_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void get_m3508_fdb(CanRxMsg * msg);

//typedef struct{
//	int32_t raw_value;   									//���������������ԭʼֵ
//	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
//	int32_t ecd_value;                       //��������������ı�����ֵ
//	int32_t diff;													//���α�����֮��Ĳ�ֵ
//	int32_t temp_count;                   //������
//	uint8_t buf_count;								//�˲�����buf��
//	int32_t ecd_bias;											//��ʼ������ֵ	
//	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
//	int32_t round_cnt;										//Ȧ��
//	int32_t filter_rate;											//�ٶ�
//	float ecd_angle;											//�Ƕ�
//}Encoder;
/*************************************************************************
                          M3508
*************************************************************************/
#endif 
