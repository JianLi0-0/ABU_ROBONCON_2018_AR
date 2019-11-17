#include "IOmanagement.h"
#include "main.h"
#define THROWING_LED_TOGGLE() GPIO_ToggleBits(GPIOC, GPIO_Pin_9)

void IO_Initialization()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��
  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_SetBits(GPIOC,GPIO_Pin_9);//GPIOF9,F10���øߣ�����
	
	
	
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);//??GPIOA,GPIOE??
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //KEY0 KEY1 KEY2????
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//??????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//??
  GPIO_Init(GPIOA, &GPIO_InitStructure);//???GPIOE2,3,4
	
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//WK_UP????PA0
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//??
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//???GPIOA0
	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//WK_UP????PA0
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//??
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//???GPIOA0

}

void THROWING_SHINING_TASK()
{
//			switch(NOW_STATE)
//	  {
//		  case RELAY1_TO_TZ1 :
//				{
//					THROWING_LED_TOGGLE();
//				}break;
//		  case RELAY1_TO_TZ2 :       /******************δ��ɽ�����********************//******************δ��ɽ�����********************/
//				{
//					THROWING_LED_TOGGLE();
//				}break;

//		  case RELAY2_TO_TZ3 :
//				{
//					THROWING_LED_TOGGLE();
//				}break;
//				
//			case RELAY2_TO_TZ2 :
//				{
//					THROWING_LED_TOGGLE();
//				}break;
//		  default:{THROW_LED_OFF();}
//		}

	switch( NOW_STATE )
	{
		case RELAY1_TO_TZ1 : 
		case RELAY1_TO_TZ2 : 
		case RELAY2_TO_TZ3 : 
		case RELAY2_TO_TZ2 :
		case TZ3_TO_RELAY2 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) // || THROW_STAGE == THROWING
			{
				THROWING_LED_TOGGLE();
			}
			else
			{
				THROW_LED_OFF();
			}
		}
		break;
		case THROW_ZONE1 : 
		case THROW_ZONE2 : 
		case THROW_ZONE3 : 
		{
			if(THROW_STAGE == THROWING) // || THROW_STAGE == THROWING  TRACKING_STATE == TRACKING_ARRIVED
			{
				THROWING_LED_TOGGLE();
			}
			else
			{
				THROW_LED_OFF();
			}
		}
		break;	
		default:{THROW_LED_OFF();};
	}

}

