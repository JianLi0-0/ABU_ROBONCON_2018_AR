#ifndef __RELAY_H
#define __RELAY_H


typedef enum
{
	RED, //交接成功
	GREEN,
	BLUE,
	NONE,
}color;

void RELAY_TASK(void);

#endif
