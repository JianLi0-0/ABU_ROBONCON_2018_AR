#ifndef __IOMANAGEMENT_H
#define __IOMANAGEMENT_H
#include "sys.h"
#define THROW_LED PCout(9)
#define THROW_LED_OFF()  GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define START_SIGNAL GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//PA5


void IO_Initialization(void);
void THROWING_SHINING_TASK(void);
#endif
