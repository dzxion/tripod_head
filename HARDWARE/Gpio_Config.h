#ifndef __Gpio_Config_H
#define __Gpio_Config_H
#include "sys.h"

#define Pitch_ON      GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define Pitch_OFF     GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define Roll_Yaw_ON   GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define Roll_Yaw_OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_14)

void Gpio_config(void);

extern u8 MPU_Flag;//陀螺数据更新标志
extern u8 Selection_axis;	//0执行Pitch代码，1执行Roll_Yaw代码

#endif
