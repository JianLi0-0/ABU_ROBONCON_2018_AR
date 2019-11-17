#ifndef __REMOTER__H
#define __REMOTER__H

typedef struct state_flag
{
	unsigned char relay_flag;    //5  B  ธ฿
	unsigned char throw_stage_flag;  //5   ตอ
	unsigned char throw_STATE_flag;  //6 F  
	unsigned char track_flag;  //7  C
	unsigned char activate_basecontrl;
	int x;
	int y;
	int z;
}state_flag;


void CHOOSE_MODE(void);

void TEST_REMOTER_CHANGE_STATE(void);
void SET_THROW_STAGE(void);
void SET_THROW_STATE(void);
void SET_TRACKING_STATE(void);
void SET_RELAY_STATE(void);


void TEST_REMOTER_CHANGE_STATE(void);
void remoter_init(void);
#endif
