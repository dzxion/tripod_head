#ifndef __Motor_Control_H
#define __Motor_Control_H
#include "sys.h" 

extern u8 Get_USART3_Buff[32];
extern u8 Fd_InsData_State;
extern u8 Tick12ms;

typedef struct//Fd_InsData_t;
{
	s16 RollRate;
	s16 PitchRate;
	s16 YawRate;   
	s16 XAccel;
	s16 YAccel;
	s16 ZAccel;
	s16 RollAngle;
	s16 PitchAngle;
	s16 YawAngle;   
	s16 HeadingAngle;    
	u16 AccelZAmp;
	u16 rev; 
}Fd_InsData_t __attribute__((aligned(4))); 

typedef struct
{
	float roll;
	float yaw;
	u32 rev1;
	u8  head_a; 
	u8  head_b; 
	u8  cmd; 
//	u8  debug_count;
	u8  check_sum; //校验和
}Send_Motor_t; 

typedef struct
{
	float roll;
	float yaw;
	u32 rev1;
	u8  head_a; 
	u8  head_b; 
	u8  cmd;
//	u8  debug_count;
	u8  check_sum; //校验和
}Get_Motor_t; 

extern Send_Motor_t Send_Motor;
extern Get_Motor_t Get_Motor;
extern Fd_InsData_t Fd_InsData;

extern float pitch_encoder, roll_encoder, yaw_encoder;

void Gimbal_Control(void);

#endif
