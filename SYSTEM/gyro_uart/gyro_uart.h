#ifndef __GYRO_UART_H
#define __GYRO_USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
	
#define EN_UART4_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

 struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};
struct SQ
{ short q[4];
};


void CopeSerial2Data(unsigned char ucData);
void GYRO_uart4_init(u32 bound);

#endif


