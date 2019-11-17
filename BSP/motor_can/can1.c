#include "can1.h"
#include "sys.h"
#include "main.h"
/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/


//Motor MOTOR[4];

/*******************************************************/
/*******************************************************/
/*******This document is wrriten by desmond lee*********/
/*******************������ѧԺ**************************/
/*******************************************************/


/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/


unsigned int CAN_Time_Out = 0;

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}

/*************************************************************************
                          CAN1_Configuration
��������ʼ��CAN1����Ϊ1M������
*************************************************************************/
void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

unsigned char can_tx_success_flag = 0;
/*************************************************************************
                          CAN1_TX_IRQHandler
������CAN1�ķ����жϺ���
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}

/****************************************************************************************
                                       ��λָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                     ģʽѡ��ָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                   �ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                   λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                  �ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}


/****************************************************************************************
                                  �����ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}


/****************************************************************************************
                                  ����λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}


/****************************************************************************************
                                  �����ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                      ����ָ��
Temp_Time��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Ctl1_Ctl2��ȡֵ��Χ��0 or 1 ������Ϊ0 or 1������Ϊ��0��Ϊ�ر�������λ��⹦��
�ر���ʾ��Ctl1��Ctl2�Ĺ��ܽ�������102 301������汾��������Ctl1_Ctl2 = 0 ����
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    if((Ctl1_Ctl2 != 0x00)&&(Ctl1_Ctl2 != 0x01))
    {
        Ctl1_Ctl2 = 0x00;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Temp_Time;
    tx_message.Data[1] = Ctl1_Ctl2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

/****************************************************************************************
                                      ���߼��
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
//    while(can_tx_success_flag == 0); //���CANоƬ��TJA1050��ע�͵�����жϡ�
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg rx_message;
    
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
		{
			Get_Encoder_Estimates_Rx(&rx_message);
		}
       
  }
}

/****************************************************************************************
                                    ����ָ��
Odrive_Control(CAN_TypeDef *CANx, u8 motor_num, int set_point, u8 control_mode)

motor_num   ȡֵ��Χ 1-12

control_mode    ȡֵ��Χ

	Position_Mode    1
	Velocity_Mode    2
	Current_Mode     3

set_point      		��λ(��������ͬ�ķ���)

	under Position_Mode    counts             ������������Ϊ1000(�ı�Ƶ��4000 counts),��setpoint 4000��ʾתһȦ
	under Velocity_Mode    counts/s						������������Ϊ1000(�ı�Ƶ��4000 counts),��setpoint 4000��ʾĿ���ٶ�Ϊ1sתһȦ
	under Current_Mode     mA									Ŀ����������ϵ����ת�س�����ΪŤ��,��С�ֱ���Ϊ10ma

*****************************************************************************************/

void Odrive_Control(CAN_TypeDef *CANx, u8 motor_num, int set_point, u8 control_mode)
{
	CanTxMsg tx_message;
	
	tx_message.StdId = (motor_num << 5) + (0x00B + control_mode); //����λΪ�����ţ�����λΪ����ģʽ��0x0C,0x0D,0x0E�ֱ���λ�� �ٶ� ����ģʽ����Ӧcontrol_modeΪ1��2��3
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	switch(control_mode)
	{
		case 1: break;
		case 2: set_point *= 100;break;
		case 3: set_point /= 10;break;
	}
	tx_message.Data[0] = set_point;	
	tx_message.Data[1] = set_point>>8;	
	tx_message.Data[2] = set_point>>16;	
	tx_message.Data[3] = set_point>>24;	
	tx_message.Data[4] = 0;
	tx_message.Data[5] = 0;
	tx_message.Data[6] = 0;
	tx_message.Data[7] = 0;

	can_tx_success_flag = 0;
	CAN_Transmit(CANx,&tx_message);
	
//	CAN_Time_Out = 0;
//	while(can_tx_success_flag == 0)
//	{
//			CAN_Delay_Us(1);
//			CAN_Time_Out++;
//			if(CAN_Time_Out>100)
//			{
//					break;
//			}
//	}
}

/****************************************************************************************
                                    ����ָ��
Get_Encoder_Estimates_Tx(CAN_TypeDef *CANx, u8 drive_num)

����������Ϊ call & respond
�˺���Ϊcall��������Ҫ�鿴�����ĵ����ţ�����
����������can�ж��н��ж�ȡ
��������Ϊλ�����ٶ�

drive_num   ȡֵ��Χ 1-12

*****************************************************************************************/

void Get_Encoder_Estimates_Tx(CAN_TypeDef *CANx, u8 drive_num)
{
    CanTxMsg tx_message;
    tx_message.StdId = (drive_num << 5) + 0x009;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0;	
    tx_message.Data[1] = 0;	
    tx_message.Data[2] = 0;	
    tx_message.Data[3] = 0;	
    tx_message.Data[4] = 0;
    tx_message.Data[5] = 0;
    tx_message.Data[6] = 0;
    tx_message.Data[7] = 0;
	
		can_tx_success_flag = 0;
    CAN_Transmit(CANx,&tx_message);
		
		CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}



/****************************************************************************************
                                    ����ָ��
Get_Encoder_Estimates_Rx(CanRxMsg * msg)

�˺�����can�����ж����Զ����ã�

Encoder_Estimates���������ݽṹ

bldc_motor[num].ActVal[0] Ϊnum��λ��
bldc_motor[num].ActVal[1] Ϊnum���ٶ�

num: 1-12
*****************************************************************************************/


typedef union Encoder_Estimates
{
	uint8_t data[8];
	float ActVal[2];
}Encoder_Estimates;

Encoder_Estimates bldc_motor[13];

extern uint32_t drive_cnt_begin,drive_cnt,rx_cnt_begin,rx_cnt;
extern u8 rx_flag;

void Get_Encoder_Estimates_Rx(CanRxMsg * msg)
{
	u8 i = 0;
	switch(msg->StdId)
	{
		case 0x020*1 + 0x009:					//0x01      0x020 * num +0x009
		{
			rx_cnt = 1000*Timer2_Count+TIM2->CNT - rx_cnt_begin;
			rx_flag = 1;
			for(i=0;i<8;i++)
			{
				bldc_motor[1].data[i] = msg->Data[i];
			} 
		};break;
		
		case 0x020*2 + 0x009:					//0x02
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[2].data[i] = msg->Data[i];
			} 
		};break;

		case 0x020*3 + 0x009:					//0x03
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[3].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*4 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[4].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*5 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[5].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*6 + 0x009:				
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[6].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*7 + 0x009:				
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[7].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*8 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[8].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*9 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[9].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*10 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[10].data[i] = msg->Data[i];
			}
		};break;

		case 0x020*11 + 0x009:				
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[11].data[i] = msg->Data[i];
			}
		};break;
		
		case 0x020*12 + 0x009:					
		{
			for(i=0;i<8;i++)
			{
				bldc_motor[12].data[i] = msg->Data[i];
			}
		};break;
		
	}
}

/*************************************************************************
                          M3508���͵���
*************************************************************************/

//void Set_Motor_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
//{
//    CanTxMsg tx_message;
//    tx_message.StdId = 0x200;
//    tx_message.IDE = CAN_Id_Standard;
//    tx_message.RTR = CAN_RTR_Data;
//    tx_message.DLC = 0x08;
//    
//    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
//    tx_message.Data[1] = (uint8_t)cm1_iq;
//    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
//    tx_message.Data[3] = (uint8_t)cm2_iq;
//    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
//    tx_message.Data[5] = (uint8_t)cm3_iq;
//    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
//    tx_message.Data[7] = (uint8_t)cm4_iq;
//    CAN_Transmit(CANx,&tx_message);
//}

union
{
	uint8_t data[12];
	float ActVal[3];
}LIDAR_POS_UNION;

union 
{
	uint8_t data[12];
	float ActVal[3];
}LIDAR_R_POS_UNION;

char ERROR_MSG[8];
u8 IS_LIDAR_RECEIVED = 0;
void GET_LIDAR_MSG(CanRxMsg * msg)
{
	u8 i = 0;
	switch(msg->StdId)
	{
		case LIDAR_XY_MSG_ID:
		{
			LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_LIDAR]);/*software watchdog*/
			IS_LIDAR_RECEIVED = 1;
			for(i=0;i<8;i++)
			{
				LIDAR_R_POS_UNION.data[i] = msg->Data[i];
			}
			LIDAR_POSITION.X_POS = LIDAR_R_POS_UNION.ActVal[0];   //ʵ�ʰ�װ��ԭ�趨������ת��180�ȣ��Ӹ���
			LIDAR_POSITION.Y_POS = LIDAR_R_POS_UNION.ActVal[1];
		};break;
		case LIDAR_ANGLE_MSG_ID:
		{
			LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_LIDAR]);/*software watchdog*/
			for(i=0;i<4;i++)
			{
				LIDAR_R_POS_UNION.data[i+8] = msg->Data[i];
			}
			LIDAR_POSITION.ANGLE_POS = LIDAR_R_POS_UNION.ActVal[2];  //ʹ�������Ƿ��ؽǶ�
		};break;
		
		case LIDAR_ERROR_MSG_ID:
		{
			for(i=0;i<8;i++)
			{
				ERROR_MSG[i] = msg->Data[0];
			}
		};break;
	}
}

void SEND_LIDAR_XY_MSG(CAN_TypeDef *CANx, float pos_x, float pos_y)
{
	CanTxMsg tx_message;
	tx_message.StdId = MAIN_CONTROLLER_XY_MSG_ID;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	LIDAR_POS_UNION.ActVal[0] = pos_x;
	LIDAR_POS_UNION.ActVal[1] = pos_y;
	int i=0;
	for(i=0;i<8;i++)
	{
		tx_message.Data[i] = LIDAR_POS_UNION.data[i];
	}
 CAN_Transmit(CANx,&tx_message);

}

void SEND_LIDAR_ANGLE_MSG(CAN_TypeDef *CANx, float pos_angle)
{
	CanTxMsg tx_message;
	tx_message.StdId =  MAIN_CONTROLLER_ANGLE_MSG_ID;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	LIDAR_POS_UNION.ActVal[2] = pos_angle;
	
	int i=0;
	for(i=0;i<4;i++)
	{
		tx_message.Data[i] = LIDAR_POS_UNION.data[i+8];
	}
  CAN_Transmit(CANx,&tx_message);
}

//void SEND_LIDAR_ERROR_MSG(CAN_TypeDef *CANx, u8 start)
//{
//	CanTxMsg tx_message;
//	tx_message.StdId =  LIDAR_ERROR_MSG_ID;
//	tx_message.IDE = CAN_Id_Standard;
//	tx_message.RTR = CAN_RTR_Data;
//	tx_message.DLC = 0x08;
//	int i=0;
//	for(i=0;i<8;i++)
//	{
//		tx_message.Data[i] = 'G';
//	}
//  CAN_Transmit(CANx,&tx_message);
//}
/**/
