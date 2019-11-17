#include "exti.h"


u32 TIME_ISR_CNT=0;
unsigned short int PPM_Sample_Cnt=0;
unsigned short int PPM_Isr_Cnt=0;
u32 Last_PPM_Time=0;
u32 PPM_Time=0;
u16 PPM_Time_Delta=0;
u16 PPM_Time_Max=0;
unsigned short int PPM_Start_Time=0;
unsigned short int PPM_Finished_Time=0;
unsigned short int PPM_Is_Okay=0;
unsigned short int PPM_Databuf[8]={0};
extern uint32_t Timer2_Count;
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
//ϵͳ����ʱ���ȡ����λus
		
		/**********************/
    Last_PPM_Time=PPM_Time;
    PPM_Time=1000*Timer2_Count+TIM2->CNT;//us
		/**********************/
		
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPM�жϽ����ж�
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
   //PPM������ʼ
    if(PPM_Is_Okay==1)
    {
    PPM_Sample_Cnt++;
    //��Ӧͨ��д�뻺����
    PPM_Databuf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
    //���ν�������
      if(PPM_Sample_Cnt>=8)
        PPM_Is_Okay=0;
    }
    if(PPM_Time_Delta>=2050)//֡������ƽ����2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }
  }
  EXTI_ClearITPendingBit(EXTI_Line3);
}
	   
//�ⲿ�жϳ�ʼ������
//��ʼ��PE2~4,PA0Ϊ�ж�����.
void PPM_EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);//PA0 ���ӵ��ж���0
	
  /* ����EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
  EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}












