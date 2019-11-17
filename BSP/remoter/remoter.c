#include "main.h"

u8 key =0;

extern unsigned short int PPM_Databuf[8];
extern unsigned short int PPM_Is_Okay;


state_flag STATE_flag;
#define USING_VISION_SWITCH_STATE 1
#define USING_COLOR_SENSORS_SWITCH_STATE 0

u8 ZONE_TO_GO = 1;


void CHOOSE_MODE()
{
	static u8 cnt=0;
	LED0 = 0;
	LED1 = 0;
//	while(cnt<100)
//	{
//	SEND_LIDAR_XY_MSG(CAN1, -10000.0, -10000.0);
//	SEND_LIDAR_ANGLE_MSG(CAN1, -10000.0);
//	delay_ms(1);
//	cnt++;
//	}
	 
//	while(ZONE_TO_GO == 0)
//	{
//		key=KEY_Scan(0);
//		if(key == 3)
//		{
//			ZONE_TO_GO = 1;
//		}
//		else if(key == 2)
//		{
//			ZONE_TO_GO = 2;
//		}
//		else if(key == 1)
//		{
//			ZONE_TO_GO = 3;
//		}
//		Supervise();
//		delay_ms(1);
//	}
	LED0 = 1;
//	while(!START_SIGNAL)
//	{
//		Supervise();
//		delay_ms(1);
//	};
	LED0 = 0;
	LED1 = 0;
}


/* 1:suceeded  2:failed  0:none */
void TEST_REMOTER_CHANGE_STATE()
{
	#if USING_VISION_SWITCH_STATE
	if(VISION_MES=='A')
	{
		STATE_flag.relay_flag = 1;
	}
//		else if(VISION_MES=='B')
//		{
//			STATE_flag.throw_STATE_flag = 1;
//		}
//		else if(VISION_MES=='C')
//		{
//			STATE_flag.throw_STATE_flag = 2;
//		}
	else
	{
	 STATE_flag.relay_flag = 0;
	 STATE_flag.throw_STATE_flag = 0;
	}
	VISION_MES = '0';
	if(STATE_flag.relay_flag == 1 && RELAY_STATE == RELAY_WAITING)
	{
		RELAY_STATE = RELAY_GOING;  // DOCKING_STATE = DOCKING_GOING;   //5-11 添加waitting
	}
	#endif
		
		
		
	#if DEBUG_REMOTER
		
	if(PPM_Is_Okay == 0)///////////////修改条件//////////////////
	{
		if(PPM_Databuf[4]>=1700)  STATE_flag.relay_flag = 1;  //交接开始
		else STATE_flag.relay_flag = 0;  //5

		if(PPM_Databuf[5]<=1300)  STATE_flag.throw_STATE_flag = 2;   //投射是否成功
		else if(PPM_Databuf[5]>=1700)  STATE_flag.throw_STATE_flag = 1;
		else STATE_flag.throw_STATE_flag = 0;  
		
		if(PPM_Databuf[7]>=1700)  STATE_flag.activate_basecontrl = 1;
		else STATE_flag.activate_basecontrl = 0;	
		
		/*************************************************************/
		if(PPM_Databuf[0]<=1450 || PPM_Databuf[0]>=1590)  STATE_flag.x = PPM_Databuf[0] - 1520 ;
		else STATE_flag.x = 0;  

		if(PPM_Databuf[1]<=1450 || PPM_Databuf[1]>=1590)  STATE_flag.y = PPM_Databuf[1] - 1520;
		else STATE_flag.y = 0;

		if(PPM_Databuf[3]<=1450 || PPM_Databuf[3]>=1590)  STATE_flag.z = PPM_Databuf[3] - 1520;
		else STATE_flag.z = 0;
		/*************************************************************/
	}
	
	if(STATE_flag.activate_basecontrl == 0) //向下打，手柄控制交接状态
	{
		if(STATE_flag.relay_flag == 1 && RELAY_STATE == RELAY_WAITING)
		{
			RELAY_STATE = RELAY_GOING;  // DOCKING_STATE = DOCKING_GOING;   //5-11 添加waitting
		}
		else if(STATE_flag.throw_STATE_flag == 1 && THROW_STATE == THROW_STATE_WAITING)
		{
			THROW_STATE = THROW_SUCCEEDED;
		}
		else if(STATE_flag.throw_STATE_flag == 2 && THROW_STATE == THROW_STATE_WAITING)
		{
			THROW_STATE = THROW_FAILED;
		}
  }
	else   //向上打，手柄控制底盘电机
	{
		ROBOT_STATE.Vx = -STATE_flag.x*8;
		ROBOT_STATE.Vy = STATE_flag.y*8;
		ROBOT_STATE.Vz = STATE_flag.z*8;
	}
		
	#endif
	
		
		
		
		
		
		#if DEBUG_ROBOT_BASE
		if(RELAY_STATE == RELAY_WAITING)
		{
			RELAY_STATE = RELAY_GOING;  // DOCKING_STATE = DOCKING_GOING;   //5-11 添加waitting
		}
		#endif
		
	  #if DEBUG_AUTOROBOT_NOESELF
		if(RELAY_STATE == RELAY_WAITING)
		{
			RELAY_STATE = RELAY_GOING;  // DOCKING_STATE = DOCKING_GOING;   //5-11 添加waitting
		}
		#endif
		


	
}

void remoter_init()
{
  PPM_EXTIX_Init();
}

void STARTUP_MODE_CHOOSE()
{
//	if(正常重启)
//		NOW_STATE = STZ_TO_RELAY2;
//	if(TZ2重启)
//		NOW_STATE = STZ_TO_RELAY2;
	
}

/**************阶段*************************/
void SET_THROW_STAGE()
{
	THROW_STAGE = THROW_ENDED;
}
/***************状态************************/
void SET_THROW_STATE()
{
	THROW_STATE = THROW_SUCCEEDED;
}
/***************************************/
void SET_TRACKING_STATE()
{
	TRACKING_STATE = TRACKING_UNKNOWN;
}

void SET_RELAY_STATE()
{
	RELAY_STATE = RELAY_SUCCEEDED;
}

