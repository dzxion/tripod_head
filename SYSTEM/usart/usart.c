#include "app.h"

u8 U2_IDLE_Flag = 0;
u8 U3_IDLE_Flag = 0;

fifo_buffer_t Rx_Buffer;

 uint8_t tx_buffer[BUFFER_SIZE];
 uint8_t rx_buffer[BUFFER_SIZE];
 uint8_t Cam_buffer[BUFFER_SIZE];

static uint8_t Tx1_buffer[BUFFER_SIZE];

uint8_t Tx3_buffer[BUFFER_SIZE];
uint8_t Rx3_buffer[BUFFER_SIZE];

//void Dma_rx1(void)
//{
//	DMA_InitTypeDef DMA_InitStructure;
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
//  DMA_DeInit(DMA1_Channel5);   
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->RDR; 
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Tx1_buffer[0]; 
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
//	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
//	DMA_Init(DMA1_Channel5, &DMA_InitStructure);  
//	USART_DMACmd(USART1,USART_DMAReq_Rx, ENABLE); 
////	DMA_ClearFlag(UART3_RX_DMA_FLAGS);
//  DMA_Cmd(DMA1_Channel5,ENABLE);	
//}

void Dma_tx1(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
  DMA_DeInit(DMA1_Channel4);   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Tx1_buffer[0]; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure); 	
} 

void Dma_rx2(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
  DMA_DeInit(DMA1_Channel6);   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->RDR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&rx_buffer[0]; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);  
	USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE); 
//	DMA_ClearFlag(UART3_RX_DMA_FLAGS);
  DMA_Cmd(DMA1_Channel6,ENABLE);	
}

void Dma_tx2(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
  DMA_DeInit(DMA1_Channel7);   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->TDR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&tx_buffer[0]; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_Init(DMA1_Channel7, &DMA_InitStructure); 	
} 

void Dma_rx3(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
  DMA_DeInit(DMA1_Channel3);   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->RDR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Rx3_buffer[0]; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);  
	USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE); 
  DMA_Cmd(DMA1_Channel3,ENABLE);	
}

void Dma_tx3(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
  DMA_DeInit(DMA1_Channel2);   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->TDR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Tx3_buffer[0]; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_Init(DMA1_Channel2, &DMA_InitStructure); 	
} 

void USART1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;                                            //??GPIO??????
    USART_InitTypeDef USART_InitStructure;                                       //??USART??????
	
	  /*??USART1?????*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,GPIO_AF_7);
//	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,GPIO_AF_7);
	  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //10-T ,11-R
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //10-T ,11-R
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOB, &GPIO_InitStructure);	

    /*配置USART3的模式*/
    USART_InitStructure.USART_BaudRate = bound;                                  //设置USART的波特率为115200
    USART_InitStructure.USART_Parity = USART_Parity_No;                           //??USART?????None
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                   //??USART?????8bit
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                        //??USART?????1
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
    USART_InitStructure.USART_Mode = USART_Mode_Tx;               //??USART??????????
    USART_Init(USART1, &USART_InitStructure);                                 //初始化USART1                                                        	
		USART_Cmd(USART1, ENABLE);                                                     //使能USART	
		Dma_tx1();
}

void USART2_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;                                            //定义GPIO初始化结构体
    USART_InitTypeDef USART_InitStructure;                                       //定义USART初始化结构体
	
	  /*配置USART1相应的时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_7);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_7);
	  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //2-T ,3-R
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //2-T ,3-R
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);		

    /*配置USART2的模式*/
    USART_InitStructure.USART_BaudRate = bound;                                  //设置USART的波特率为115200
    USART_InitStructure.USART_Parity = USART_Parity_No;                           //设置USART的校验位为None
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                   //设置USART的数据位为8bit
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                        //设置USART的停止位为1
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //失能硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;               //设置USART的模式为发送接收模式
    USART_Init(USART2, &USART_InitStructure);                                     //初始化USART1
                                                        
		/* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//		USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE); 
//		USART_ITConfig(USART2,USART_IT_TXE,DISABLE); 
		USART_ITConfig(USART2,USART_IT_IDLE,ENABLE); 
	
		USART_Cmd(USART2, ENABLE);                                                     //使能USART
    USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE); //????1?DMA??
		
		Dma_rx2();
		Dma_tx2();
		
		Rx_Buffer.data = Cam_buffer;
		Rx_Buffer.size = BUFFER_SIZE;
		Rx_Buffer.read_index = 0;
		Rx_Buffer.write_index = 0;
}

void USART3_init(u32 bound)
{
	  NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;                                            //??GPIO??????
    USART_InitTypeDef USART_InitStructure;                                       //??USART??????
	
	  /*??USART1?????*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_7);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_7);
	  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //10-T ,11-R
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //10-T ,11-R
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

    /*配置USART3的模式*/
    USART_InitStructure.USART_BaudRate = bound;                                  //设置USART的波特率为115200
    USART_InitStructure.USART_Parity = USART_Parity_No;                           //??USART?????None
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                   //??USART?????8bit
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                        //??USART?????1
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;               //设置USART的模式为发送接收模式
    USART_Init(USART3, &USART_InitStructure);                                 //初始化USART1                                                        	
		USART_Cmd(USART3, ENABLE);                                                     //使能USART	
		
		/* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//		USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
//		USART_ITConfig(USART3,USART_IT_RXNE,ENABLE); 
//		USART_ITConfig(USART3,USART_IT_TXE,DISABLE); 
		USART_ITConfig(USART3,USART_IT_IDLE,ENABLE); 
	
		USART_Cmd(USART3, ENABLE);                                                     //使能USART
    USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE); //????1?DMA??
		
		Dma_rx3();
		Dma_tx3();
}

uint16_t Serial_available(void)//获取长度
{
    uint16_t len = 0;
    if (Rx_Buffer.read_index > Rx_Buffer.write_index)
    {
        len = Rx_Buffer.size + Rx_Buffer.write_index - Rx_Buffer.read_index;
    }
    else if (Rx_Buffer.read_index  < Rx_Buffer.write_index)
    {
        len = Rx_Buffer.write_index - Rx_Buffer.read_index;
    }
    return len;
}

uint8_t Serial_read_char(void)
{
    uint8_t ch = 0;
    ch = Rx_Buffer.data[Rx_Buffer.read_index];
    Rx_Buffer.read_index = (Rx_Buffer.read_index + 1) % Rx_Buffer.size;
    return ch;
}

uint16_t Serial_read(uint8_t *buffer, uint16_t length)//读取队列数据
{
    uint16_t i = 0;
	
    for (i = 0; i < length; i++)
    {
        buffer[i] = Rx_Buffer.data[Rx_Buffer.read_index];
        Rx_Buffer.read_index = (Rx_Buffer.read_index + 1) % Rx_Buffer.size;
    }
    return i;
}

uint32_t UART1_SendDataDMA(uint8_t *data, uint16_t len)
{  
	memcpy((void*)Tx1_buffer,data,len);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel4,len);
	DMA_Cmd(DMA1_Channel4, ENABLE);
  return len;
}

uint32_t UART2_SendDataDMA(uint8_t *data, uint16_t len)
{  
	memcpy((void*)tx_buffer,data,len);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(DMA1_Channel7, DISABLE);      
	DMA_SetCurrDataCounter(DMA1_Channel7,len);
	DMA_Cmd(DMA1_Channel7, ENABLE);
  return len;
}

uint32_t UART3_SendDataDMA(uint8_t *data, uint16_t len)
{  
	memcpy((void*)Tx3_buffer,data,len);
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel2,len);
	DMA_Cmd(DMA1_Channel2, ENABLE);
  return len;
}

u16 Get_CamData(void)
{
	u16 len = 0;
	uint16_t i = 0;
	if(U2_IDLE_Flag == 1)
	{	
    len = BUFFER_SIZE - DMA1_Channel6->CNDTR;		
		for(i=0;i<len;i++)
		{
			Rx_Buffer.data[Rx_Buffer.write_index] = rx_buffer[i];	
      Rx_Buffer.write_index = (Rx_Buffer.write_index + 1) % Rx_Buffer.size;					
		}
	
		DMA_Cmd(DMA1_Channel6, DISABLE);//DISABLE //ENABLE	
		memset(rx_buffer,0,sizeof(char)*BUFFER_SIZE);//清空接收数据包
		
		DMA1_Channel6->CNDTR = BUFFER_SIZE;
		USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);//DISABLE //ENABLE
		U2_IDLE_Flag = 0;
	}
	return len;
}

void USART2_IRQHandler(void)//rx_buffer
{
  if(USART_GetITStatus(USART2, USART_IT_IDLE)  != RESET)  
	{ 
    USART_ClearFlag(USART2, USART_IT_IDLE);		
		USART_ClearFlag(USART2, USART_FLAG_IDLE);
		U2_IDLE_Flag = 1;			 
	}     
}

void USART3_IRQHandler(void)//rx_buffer
{
  u16 len = 0;
  if(USART_GetITStatus(USART3, USART_IT_IDLE)  != RESET)  
	{ 					
		USART_ClearFlag(USART3, USART_FLAG_IDLE);
		len = BUFFER_SIZE - DMA1_Channel3->CNDTR;		
    memcpy(Get_USART3_Buff,Rx3_buffer,len);
		DMA_Cmd(DMA1_Channel3, DISABLE);//DISABLE //ENABLE
		DMA1_Channel3->CNDTR = BUFFER_SIZE;
		DMA_Cmd(DMA1_Channel3, ENABLE);//DISABLE //ENABLE		
	
		U3_IDLE_Flag = 1;					
	}     
}


