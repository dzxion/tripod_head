#include "led.h"

u16 RLedCount = 0;
u16 GLedCount = 0;

void LED_Init(void)
{ 
  GPIO_InitTypeDef	GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_SetBits(GPIOC,GPIO_Pin_14);
}
 
void R_LED_Status(u8 LedData)
{
	if(LedData == 0)//ºìµÆÃð
	{
		RED_LED_1;
		RLedCount = 0;
	}
	else if(LedData == 1)//ºìµÆ³¤ÁÁ
	{
		RED_LED_0;
		RLedCount = 0;
	}
	else if(LedData == 2)//ºìµÆ¿ìÉÁ
	{
		if(RLedCount < 200)
		{
			RED_LED_0;
		}
		else if(RLedCount == 200)
		{
			RED_LED_1;
		}
		else if(RLedCount >= 400)
		{
			RLedCount = 0;
		}
	}
	else if(LedData == 3)//ºìµÆµ¥ÉÁ
	{
		if(RLedCount < 80)
		{
			RED_LED_0;
		}
		else if(RLedCount == 80)
		{
			RED_LED_1;
		}
		else if(RLedCount >= 2000)
		{
			RLedCount = 0;
		}
	}
	else if(LedData == 4)//ºìµÆË«ÉÁ
	{
		if(RLedCount < 80)
		{
			RED_LED_0;
		}
		else if(RLedCount == 80)
		{
			RED_LED_1;
		}
		else if(RLedCount == 250)
		{
			RED_LED_0;
		}
		else if(RLedCount == 330)
		{
			RED_LED_1;
		}
		else if(RLedCount >= 2000)
		{
			RLedCount = 0;
		}
	}
	RLedCount++;
}

void G_LED_Status(u8 LedData)
{
	if(LedData == 0)//ºìµÆÃð
	{
		GREEN_LED_1;
		GLedCount = 0;
	}
	else if(LedData == 1)//ºìµÆ³¤ÁÁ
	{
		GREEN_LED_0;
		GLedCount = 0;
	}
	else if(LedData == 2)//ºìµÆ¿ìÉÁ
	{
		if(GLedCount < 200)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 200)
		{
			GREEN_LED_1;
		}
		else if(GLedCount >= 400)
		{
			GLedCount = 0;
		}
	}
	else if(LedData == 3)//ºìµÆµ¥ÉÁ
	{
		if(GLedCount < 80)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 80)
		{
			GREEN_LED_1;
		}
		else if(GLedCount >= 2000)
		{
			GLedCount = 0;
		}
	}
	else if(LedData == 4)//ºìµÆË«ÉÁ
	{
		if(GLedCount < 80)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 80)
		{
			GREEN_LED_1;
		}
		else if(GLedCount == 250)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 330)
		{
			GREEN_LED_1;
		}
		else if(GLedCount >= 2000)
		{
			GLedCount = 0;
		}
	}
	else if(LedData == 5)//ºìµÆÈýÉÁ
	{
		if(GLedCount < 80)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 80)
		{
			GREEN_LED_1;
		}
		else if(GLedCount == 250)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 330)
		{
			GREEN_LED_1;
		}
		
		else if(GLedCount == 500)
		{
			GREEN_LED_0;
		}
		else if(GLedCount == 580)
		{
			GREEN_LED_1;
		}
		
		else if(GLedCount >= 2000)
		{
			GLedCount = 0;
		}
	}
	
	GLedCount++;
}

