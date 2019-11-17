#include "main.h"

/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

int16_t  pitch_ecd_bias =6000;
int16_t  yaw_ecd_bias  = 5000;
Motor MOTOR[4];

void CAN2_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_12 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
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
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}



void Set_Motor_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}



void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //������������ݴ���
        get_m3508_fdb(&rx_message);
    }
}

void get_m3508_fdb(CanRxMsg * msg)
{
	switch(msg->StdId)
	{
		case CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID:
		{
			LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_MOTOR1]);/*software watchdog*/
			MOTOR[0].angle = (msg->Data[0]<<8)|msg->Data[1];
			MOTOR[0].inner_rpm = (msg->Data[2]<<8)|msg->Data[3];
			MOTOR[0].current = (msg->Data[4]<<8)|msg->Data[5];
		};break;
		case CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID:
		{
			LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_MOTOR2]);/*software watchdog*/
			MOTOR[1].angle = (msg->Data[0]<<8)|msg->Data[1];
			MOTOR[1].inner_rpm = (msg->Data[2]<<8)|msg->Data[3];
			MOTOR[1].current = (msg->Data[4]<<8)|msg->Data[5];
		};break;
		case CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID:
		{
			LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_MOTOR3]);/*software watchdog*/
			MOTOR[2].angle = (msg->Data[0]<<8)|msg->Data[1];
			MOTOR[2].inner_rpm = (msg->Data[2]<<8)|msg->Data[3];
			MOTOR[2].current = (msg->Data[4]<<8)|msg->Data[5];
		};break;
		case CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID:
		{
			MOTOR[3].angle = (msg->Data[0]<<8)|msg->Data[1];
			MOTOR[3].inner_rpm = (msg->Data[2]<<8)|msg->Data[3];
			MOTOR[3].current = (msg->Data[4]<<8)|msg->Data[5];
		};break;
	}
	
}
