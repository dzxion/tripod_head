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
	TIM_TimeBaseStructure.TIM_Period=arr;       //��ʱ������
	TIM_TimeBaseStructure.TIM_Prescaler=psc;     //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷ�Ƶϵ��������Ƶ������TIM_CKD_DIV1  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ������ģʽ  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//����ΪPWMģʽ1  
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                

	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);//��ʱ��2��3ͨ������  
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//��ʱ��2��3ͨ������  ͨ��3��PA2
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIM2���ؼĴ���ARR  

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
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
	                                                                	 //����TIM3��CH2�����PWMͨ����LED��ʾ
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICFilter = 0x02;	  //IC1F=0000 ���������˲��� ���˲�
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);

	TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2 �ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ� 2 ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //�����ȼ� 0 ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); //��ʼ������ NVIC �Ĵ���
	
	TIM_ITConfig(TIM2, TIM_IT_CC2,ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�		
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //����жϱ�־λ
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
}

void TIM15_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM15_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
	                                                                	 //����TIM3��CH2�����PWMͨ����LED��ʾ
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM15_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM15_ICInitStructure.TIM_ICFilter = 0x02;	  //IC1F=0000 ���������˲��� ���˲�
	TIM15_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM15_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM15_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInit(TIM15,&TIM15_ICInitStructure);

	TIM_PWMIConfig(TIM15, &TIM15_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM15, TIM_TS_TI1FP1);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM15, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM15, TIM_MasterSlaveMode_Enable); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn; //TIM2 �ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ� 2 ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //�����ȼ� 0 ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); //��ʼ������ NVIC �Ĵ���
	
	TIM_ITConfig(TIM15, TIM_IT_CC1,ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�		
	TIM_ClearITPendingBit(TIM15, TIM_IT_CC1); //����жϱ�־λ
	TIM_Cmd(TIM15, ENABLE);  //ʹ��TIMx����
}

void TIM4_Int_Init(u16 arr,u16 psc)  
{
 	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;   
	NVIC_InitTypeDef  NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);  //Timer4ʱ��ʹ��
	TIM_DeInit(TIM4);                              //��λTIM2��ʱ�� 
	TIM_TimeBaseStructure.TIM_Period=arr;       //��ʱ������
	TIM_TimeBaseStructure.TIM_Prescaler=psc;     //Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision=0;      //TIM4ʱ�ӷ�Ƶ,0��ʾ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//��ʱ������Ϊ���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);     
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);         //�����ʱ��2�������־λ
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);      //ʹ�ܶ�ʱ��2����ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //PPP�ⲿ�ж��� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4, ENABLE);       	//��ʱ��4ʹ��
}

u16 duty,period;
u16 duty_data;
u16 duty_data_Y;

ThetaOffset_t ThetaOffset;
Get_Encoder_t Get_Encoder;

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//����1���������¼�
	{	
		duty = TIM_GetCapture1(TIM2); 				//�ɼ�ռ�ձ�		
		period	=	TIM_GetCapture2(TIM2);     //�ɼ�����
		
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
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //����жϱ�־λ
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM15, TIM_IT_CC1) != RESET)//����1���������¼�
	{	
		duty = TIM_GetCapture2(TIM15); 				//�ɼ�ռ�ձ�		
		period	=	TIM_GetCapture1(TIM15);     //�ɼ�����
		duty_data_Y = (u16)(514.0f*duty/period-1.0f);
		Get_Encoder.Angle_Y = 0.703125f * ((u16)(duty_data_Y + 102)%512);
	}
	TIM_ClearITPendingBit(TIM15, TIM_IT_CC1); //����жϱ�־λ
}

u8 TIM4_Flag = 0;
void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4 ,TIM_FLAG_Update); //�������жϱ�־λ
	TIM4_Flag = 1;
}

