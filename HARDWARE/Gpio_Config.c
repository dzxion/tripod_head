#include "app.h"

u8 Selection_axis = 0; 

void Gpio_Exti(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//??SYSCFG??
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);//
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;//
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//
  EXTI_Init(&EXTI_InitStructure);// 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
  NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0;//
  NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void Gpio_config(void)
{
  GPIO_InitTypeDef	GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //ROLL_YAW 和 PITCH 运行代码通过PB12引脚选择
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	
	delay_ms(100);
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0)Selection_axis = 1; else Selection_axis = 0;//0执行Pitch代码，1执行Roll_Yaw代码
	
  if(Selection_axis == 0)
	{		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		Gpio_Exti();
	}	
	else if(Selection_axis == 1)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);	
	}
	
}


u8 MPU_Flag = 0;//定时器500us标志

void EXTI0_IRQHandler(void) //  EXTI9_5_IRQHandler
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)//中断发生时  ==RESET//中断没有发生时
	{	
    EXTI_ClearITPendingBit(EXTI_Line0);//清除外部中断	
		MPU_Flag = 1;
	}
}
