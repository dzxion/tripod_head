#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

typedef struct
{
	float Angle_P;
	float Angle_R;
	float Angle_Y;
}Get_Encoder_t;

typedef struct
{
	u16 P;
	u16 R;
	u16 Y;
}ThetaOffset_t; //��Ƕ���ƫ

extern Get_Encoder_t Get_Encoder;
extern ThetaOffset_t ThetaOffset;

void Heating_Drive(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM15_PWM_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM8_Cap_Init(u16 arr, u16 psc);

extern u16 duty_data;
extern u16 duty_data_Y;
extern u8 TIM4_Flag;//��ʱ��500us��־

#endif
