#include "main.h"
//uint32_t time_test_start;
//uint32_t time_test_interval;
int set_point=4000;
u8 mode=2;
uint32_t drive_cnt_begin,drive_cnt,rx_cnt_begin,rx_cnt;
u8 rx_flag = 1;
int main(void)
{
 	delay_init(168);  //初始化延时函数
	BSP_Init();
	
	while(1)
	{
		drive_cnt_begin = 1000*Timer2_Count+TIM2->CNT;
		
		if(rx_flag)
		{
			rx_cnt_begin = 1000*Timer2_Count+TIM2->CNT;
			rx_flag = 0;
		}
		
		Odrive_Control(CAN1, 1, set_point, mode); //速度模式，1号电机，速度大小4000 counts/s
		Odrive_Control(CAN1, 3, set_point, mode); //速度模式，1号电机，速度大小4000 counts/s
		
		Get_Encoder_Estimates_Tx(CAN1, 1);				//反馈call
		Get_Encoder_Estimates_Tx(CAN1, 3);				//反馈call
		
		drive_cnt = 1000*Timer2_Count+TIM2->CNT - drive_cnt_begin;
		
		delay_us(400);
		
		

//		if(Count_1ms>=1)
//		{	
//			Count_1ms = 0;
//			Task_1000HZ();
//		}
//		if(Count_2ms>=2)
//		{
//			Count_2ms = 0;
//			Task_500HZ();
//		}
//		if(Count_4ms>=4)
//		{
//			Count_4ms = 0;
//			Task_250HZ();
//		}
//		if(Count_5ms>=5)
//		{
//			Count_5ms = 0;
//			Task_200HZ();
//		}
//		if(Count_10ms>=10)
//		{
//			Count_10ms = 0;
//			Task_100HZ();
//		}
//		if(Count_20ms>=20)
//		{
//			Count_20ms = 0;
//			Task_50HZ();
//		}
//		if(Count_50ms>=50)
//		{
//			Count_50ms = 0;
//			Task_20HZ();
//		}
//		if(Count_100ms>=100)
//		{
//			Count_100ms = 0;
//			Task_10HZ();
//		}
//		if(Count_200ms>=200)
//		{
//			Count_200ms = 0;
//			Task_5HZ();
//		}
//		if(Count_1000ms>=1000)
//		{
//			Count_1000ms = 0;
//			Task_1HZ();
//		}
		
	}
}
