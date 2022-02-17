#ifndef __Gpio_Config_H
#define __Gpio_Config_H
#include "sys.h"

#define Pitch_ON      GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define Pitch_OFF     GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define Roll_Yaw_ON   GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define Roll_Yaw_OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_14)

void Gpio_config(void);

extern u8 MPU_Flag;//�������ݸ��±�־
extern u8 Selection_axis;	//0ִ��Pitch���룬1ִ��Roll_Yaw����

#endif
