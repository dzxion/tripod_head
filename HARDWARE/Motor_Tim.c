#include "app.h"

#define REP_RATE 3  //中断周期1 = 20K = 50us
                    //中断周期3 = 10K = 100us
                    //中断周期5 = 6.666K = 150us

void SVPWM1_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
  TIM_OCInitTypeDef TIM1_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE );

  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
  TIM1_TimeBaseStructure.TIM_Prescaler = 0;
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM1_TimeBaseStructure.TIM_Period = TS;
  TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
  
	  // Initial condition is REP=0 to set the UPDATE only on the underflow
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
	
  TIM_OCStructInit(&TIM1_OCInitStructure);
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                 
  TIM1_OCInitStructure.TIM_Pulse = TIM1->ARR/2; //dummy value
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;         
  
  TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
  TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_6);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 ;                       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//DISABLE ENABLE
  NVIC_Init(&NVIC_InitStructure);
  
	TIM_ITConfig(TIM1, TIM_IT_Update,DISABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

void SVPWM8_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM8_TimeBaseStructure;
  TIM_OCInitTypeDef TIM8_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );

  TIM_DeInit(TIM8);
  TIM_TimeBaseStructInit(&TIM8_TimeBaseStructure);
  TIM8_TimeBaseStructure.TIM_Prescaler = 0;
  TIM8_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM8_TimeBaseStructure.TIM_Period = TS;
  TIM8_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseStructure);
  
	  // Initial condition is REP=0 to set the UPDATE only on the underflow
  TIM8_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
  TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseStructure);
	
  TIM_OCStructInit(&TIM8_OCInitStructure);
  TIM8_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM8_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM8_OCInitStructure.TIM_Pulse = TIM8->ARR/2; //dummy value
  TIM8_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         
  TIM8_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;         
  
  TIM_OC1Init(TIM8, &TIM8_OCInitStructure); 
  TIM_OC2Init(TIM8, &TIM8_OCInitStructure);
  TIM_OC3Init(TIM8, &TIM8_OCInitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_10);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_10);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 ;                       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//DISABLE ENABLE
  NVIC_Init(&NVIC_InitStructure);
  
	TIM_ITConfig(TIM8, TIM_IT_Update,DISABLE);
	
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
  TIM_Cmd(TIM8, ENABLE);
	
	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;
	TIM8->CCR3 = 0;
}

STM32F303_RAMFUNC void TIM1_UP_TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
	  TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}

STM32F303_RAMFUNC void TIM8_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
	{
	  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	}
}
