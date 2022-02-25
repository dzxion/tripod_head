#include "app.h"

void Heating_Drive(u16 arr,u16 psc)  
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_2);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 
	/* Time base configuration */                                            
	TIM_TimeBaseStructure.TIM_Period=arr;       //定时器周期
	TIM_TimeBaseStructure.TIM_Prescaler=psc;     //预分频数 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分频系数：不分频，或者TIM_CKD_DIV1  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数溢出模式  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//配置为PWM模式1  
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                

	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);//定时器2的3通道配置  
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//定时器2的3通道配置  通道3：PA2
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIM2重载寄存器ARR  

	TIM_Cmd(TIM3, ENABLE);       

  TIM_SetCompare1(TIM3,0);
}

void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   //使能GPIO外设和AFIO复用功能模块时钟使能
	                                                                	 //用于TIM3的CH2输出的PWM通过该LED显示
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2_ICInitStructure.TIM_ICFilter = 0x02;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);

	TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级 2 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级 0 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化外设 NVIC 寄存器
	
	TIM_ITConfig(TIM2, TIM_IT_CC2,ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断		
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //清除中断标志位
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
}

void TIM15_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM15_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   //使能GPIO外设和AFIO复用功能模块时钟使能
	                                                                	 //用于TIM3的CH2输出的PWM通过该LED显示
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM15_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM15_ICInitStructure.TIM_ICFilter = 0x02;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM15_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM15_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM15_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInit(TIM15,&TIM15_ICInitStructure);

	TIM_PWMIConfig(TIM15, &TIM15_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM15, TIM_TS_TI1FP1);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM15, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM15, TIM_MasterSlaveMode_Enable); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn; //TIM2 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级 2 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级 0 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化外设 NVIC 寄存器
	
	TIM_ITConfig(TIM15, TIM_IT_CC1,ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断		
	TIM_ClearITPendingBit(TIM15, TIM_IT_CC1); //清除中断标志位
	TIM_Cmd(TIM15, ENABLE);  //使能TIMx外设
}

void TIM4_Int_Init(u16 arr,u16 psc)  
{
 	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;   
	NVIC_InitTypeDef  NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);  //Timer4时钟使能
	TIM_DeInit(TIM4);                              //复位TIM2定时器 
	TIM_TimeBaseStructure.TIM_Period=arr;       //定时器周期
	TIM_TimeBaseStructure.TIM_Prescaler=psc;     //预分频数
	TIM_TimeBaseStructure.TIM_ClockDivision=0;      //TIM4时钟分频,0表示不分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//定时器计数为向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);     
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);         //清除定时器2的溢出标志位
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);      //使能定时器2溢出中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //PPP外部中断线 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4, ENABLE);       	//定时器4使能
}

u16 duty,period;
u16 duty_data;
u16 duty_data_Y;

ThetaOffset_t ThetaOffset;
Get_Encoder_t Get_Encoder;

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
	{	
		duty = TIM_GetCapture1(TIM2); 				//采集占空比		
		period	=	TIM_GetCapture2(TIM2);     //采集周期
		
		duty_data = (u16)(514.0f*duty/period-1.0f);
		
		if(Selection_axis == 0)
		{
//      Get_Encoder.Angle_P = 0.703125f * ((u16)(duty_data + ThetaOffset.P)%512);
			Get_Encoder.Angle_P = 0.703125f * ((u16)(duty_data + 225)%512);
		}
		else if(Selection_axis == 1)
		{
      Get_Encoder.Angle_R = 0.703125f * ((u16)(duty_data + 158)%512);
		}
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //清除中断标志位
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM15, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
	{	
		duty = TIM_GetCapture2(TIM15); 				//采集占空比		
		period	=	TIM_GetCapture1(TIM15);     //采集周期
		duty_data_Y = (u16)(514.0f*duty/period-1.0f);
		Get_Encoder.Angle_Y = 0.703125f * ((u16)(duty_data_Y + 102)%512);
	}
	TIM_ClearITPendingBit(TIM15, TIM_IT_CC1); //清除中断标志位
}

u8 TIM4_Flag = 0;
void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4 ,TIM_FLAG_Update); //清除溢出中断标志位
	TIM4_Flag = 1;
}

