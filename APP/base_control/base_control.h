#ifndef __BASE_CONTROL_H
#define __BASE_CONTROL_H
#include "sys.h"


#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER          (0.86602540f) //    根号3/2   
#define L_PARAMETER          (1.0f) 

//#define amplify               10
//#define Radius               (48.2f)  //底盘半径  L=302.85 cm
//#define pulse_per_centimeter (787.5f)  ///******轮子转一圈脉冲数 n= 500*4*19=38000,轮子半径R=7.68cm,周长L=2*pi*R=50cm**  1du~0.84125cm***/
//#define cm_per_angle         (0.84125f)  
//#define dt                   (0.02f)

void Kinematic_Analysis(float Vx,float Vy,float Vz);
void World_Kinematic_Analysis(float Vx, float Vy, float Vz, float theta);
//void World_Forward_Kinematic_Analysis(float theta);
void motor_init(void);
void base_control_task(void);
void locked_rotor_current_check(void);
typedef struct R_S{
	int16_t TARGET_Vx;
  int16_t TARGET_Vy;
	int16_t TARGET_Vz;
	int16_t REAL_Vx;
  int16_t REAL_Vy;
	int16_t REAL_Vz;
  int16_t Vx;
  int16_t Vy;
	int16_t Vz;
	int16_t Last_Vx;
	int16_t Last_Vy;
	int16_t Last_Vz;
}R_S;


typedef struct M_S{
  int16_t Motor_A_SPEED;
  int16_t Motor_B_SPEED;
	int16_t Motor_C_SPEED;
	int16_t Motor_D_SPEED;
}M_S;


#endif
