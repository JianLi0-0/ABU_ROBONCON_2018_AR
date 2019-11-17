#ifndef __TCS230_H
#define __TCS230_H
#include "sys.h"

#define Red1    1
#define Green1  2
#define Blue1   3
#define Yellow1 4
#define White1  5
#define Black1  6


/*输出端口定义*/
#define TCS32_S2 PEout(0)//PE0
#define TCS32_S3 PEout(1)//PE1

/*外部函数声明*/
void TIM7_Int_Init(u16 arr,u16 psc);
void TCS230_EXTIX_Init(void);
void TCS230_Init(void);
void Colour_Init(u8 Colour);
void TCS230_Start(void);
u32 TCS230_Read_Colour(u8 Colour);
int TCS230_Distinguish(void);
#endif
