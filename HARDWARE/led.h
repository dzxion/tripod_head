#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define RED_LED_1  GPIO_SetBits(GPIOC,GPIO_Pin_14)
#define RED_LED_0  GPIO_ResetBits(GPIOC,GPIO_Pin_14)
#define GREEN_LED_1  GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define GREEN_LED_0  GPIO_ResetBits(GPIOC,GPIO_Pin_13)

void LED_Init(void);
void R_LED_Status(u8 LedData);
void G_LED_Status(u8 LedData);
		 				    
#endif
