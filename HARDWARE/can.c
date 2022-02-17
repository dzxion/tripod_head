#include "app.h"

#define CAN_BUFFER_SIZE   (128)

static Can_fifo_buffer_t CANRx_Buffer;
static uint8_t CAN_Rx_buffer[CAN_BUFFER_SIZE];
//static port_t can_port;

Can_fifo_buffer_t CANTx_Buffer;
uint8_t CAN_Tx_buffer[CAN_BUFFER_SIZE];
uint8_t TxMessage[8];

uint8_t  CanIDNum = 20;
uint32_t CANIDTOFC = 0x0; 

void CAN_Para_Init(void)
{
	CANRx_Buffer.data = CAN_Rx_buffer;
	CANRx_Buffer.size = CAN_BUFFER_SIZE;
	CANRx_Buffer.read_index = 0;
	CANRx_Buffer.write_index = 0;
  
//	can_port.available = can_serial_available;
//	can_port.read = can_serial_read;
//	can_port.read_one = can_serial_read_char;
//	can_port.write = can_serial_write;
	
	CANTx_Buffer.data = CAN_Tx_buffer;
	CANTx_Buffer.size = CAN_BUFFER_SIZE;
	CANTx_Buffer.read_index = 0;
	CANTx_Buffer.write_index = 0;
}

void StdCanSetID(u8 filterNumber, u32 id1,u32 id2)//标准帧普通过滤函数filterNumber：通道   一个通道最多支持2个id号，一共13个通道，最多只支持过滤26个id号
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = filterNumber % 14; 
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList; 
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	id1 %= (1 << 11);
	id2 %= (1 << 11);
	CAN_FilterInitStructure.CAN_FilterIdHigh = id1 << 5; 
	CAN_FilterInitStructure.CAN_FilterIdLow = CAN_ID_STD | CAN_RTR_DATA;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = id2 << 5;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_ID_STD | CAN_RTR_DATA;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; 
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/* 设置完波特率之后，要设置过滤器 */
void CanSetBaudrate(u16 baud /*KBit*/)//
{
	CAN_InitTypeDef CAN_InitStructure;
//	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;			//禁止时间触发通信模式
	CAN_InitStructure.CAN_ABOM = ENABLE;			//bxCAN进入离线状态后，就自动开启恢复过程
	CAN_InitStructure.CAN_AWUM = DISABLE;			//睡眠模式通过清除CAN_MCR寄存器的SLEEP位，由软件唤醒
	CAN_InitStructure.CAN_NART = DISABLE;			//按照CAN标准，CAN硬件在发送报文失败时会一直自动重传直到发送成功 
	CAN_InitStructure.CAN_RFLM = DISABLE;			//在接收溢出时FIFO未被锁定，当接收FIFO的报文未被读出，下一个收到的报文会覆盖原有的报文
	CAN_InitStructure.CAN_TXFP = DISABLE;			//发送FIFO优先级由报文的标识符来决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//CAN_Mode_LoopBack;//CAN硬件工作在正常模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;		//重新同步跳跃宽度1个时间单位
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;		//时间段1为3个时间单位
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;		//时间段2为2个时间单位
	if(baud == 0)
	{
		baud = CAN_BAUDRATE_500K;
	}
	CAN_InitStructure.CAN_Prescaler = 3000 / baud;	//baud = (PCLK/Prescaler)/(1+6+5), APB1=HCLK/2=36MHz
	CAN_Init(CAN1,&CAN_InitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);			//CAN FIFO0 message pending interrupt enable
}

void BSP_CAN1_FilterConfig(u16 id)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber         = 0;
	CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh         = id<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow          = 0; 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0xFFFF;  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0xFFFC;            //RTR 可以为数据帧也可以远帧
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

void CanHardwareInit(u16 CAN_BAUDRATE)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
        
  //引脚复用映射配置
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_9); //GPIOA11复用为CAN1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_9); //GPIOA12复用为CAN1
	
	CAN_Para_Init();
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	CanSetBaudrate(CAN_BAUDRATE);
	
	BSP_CAN1_FilterConfig(0);

	StdCanSetID(0, 0x500+CanIDNum, 0x500+CanIDNum);//标准帧过滤
  CANIDTOFC = 0x580 + CanIDNum;
}


uint8_t CanSend(u32 id, u8 *data, u8 size)
{
	CanTxMsg TxMessage;
	u16 i = 0;
	u8  Mailbox = CAN_TxStatus_NoMailBox;
	if(id <= 0x7FF)
	{
		TxMessage.StdId = id;
		TxMessage.IDE = CAN_ID_STD;
	}
	else
	{
		TxMessage.ExtId = id;
		TxMessage.IDE = CAN_ID_EXT;
	}
	if(size > 8)
		size = 8;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC =size;
	memcpy(TxMessage.Data, data, size);
	while( Mailbox == CAN_TxStatus_NoMailBox &&  i < 0xFFF  )
	{
			Mailbox = CAN_Transmit( CAN1 , &TxMessage );
			i++;
	}
	if(i >= 0xFFF)return 0;
	return 1;
}


uint16_t can_serial_available(void)
{
    uint16_t len = 0;
    if (CANRx_Buffer.read_index > CANRx_Buffer.write_index)
    {
        len = CANRx_Buffer.size + CANRx_Buffer.write_index - CANRx_Buffer.read_index;
    }
    else if (CANRx_Buffer.read_index  < CANRx_Buffer.write_index)
    {
        len = CANRx_Buffer.write_index - CANRx_Buffer.read_index;
    }
    return len;
}
uint8_t can_serial_read_char(void)
{
    uint8_t ch = 0;
    ch = CANRx_Buffer.data[CANRx_Buffer.read_index];
    CANRx_Buffer.read_index = (CANRx_Buffer.read_index + 1) % CANRx_Buffer.size;
    return ch;
}

uint16_t can_serial_read(uint8_t *buffer, uint16_t length)
{
    uint16_t i = 0;
	
    for (i = 0; i < length; i++)
    {
        buffer[i] = CANRx_Buffer.data[CANRx_Buffer.read_index];
        CANRx_Buffer.read_index = (CANRx_Buffer.read_index + 1) % CANRx_Buffer.size;
    }
    return i;
}

uint16_t can_tx_buf(uint8_t *buffer, uint16_t length, Can_fifo_buffer_t* Tx_Buffer)
{
    uint16_t i = 0;
	
    for (i = 0; i < length; i++)
    {
        Tx_Buffer->data[Tx_Buffer->write_index] = buffer[i];
        Tx_Buffer->write_index = (Tx_Buffer->write_index + 1) % Tx_Buffer->size;
    }
    return i;
}

uint16_t can_serial_write(uint8_t *buffer, uint16_t length)
{
    uint16_t i = 0;
	  can_tx_buf(buffer, length, &CANTx_Buffer);
    return i;
}

uint16_t can_tx_available(Can_fifo_buffer_t * Tx_Buffer)
{
    uint16_t len = 0;
    if (Tx_Buffer->read_index > Tx_Buffer->write_index)
    {
        len = Tx_Buffer->size + Tx_Buffer->write_index - Tx_Buffer->read_index;
    }
    else if (Tx_Buffer->read_index  < Tx_Buffer->write_index)
    {
        len = Tx_Buffer->write_index - Tx_Buffer->read_index;
    }
    return len;
}

uint16_t tx_len1 = 0;

void can_transmit(void)
{
    tx_len1 =  can_tx_available(&CANTx_Buffer);
	  if(tx_len1 > 0)
		{
			
			uint16_t temp;
			if(CANIDTOFC != 0)
		  {
				if(tx_len1 > 8)
				{
					temp = CANTx_Buffer.size - CANTx_Buffer.read_index;
					if(temp < 8)
					{
						memcpy(TxMessage,&(CANTx_Buffer.data[CANTx_Buffer.read_index]), temp);
						memcpy(&TxMessage[temp],&(CANTx_Buffer.data[0]) ,8-temp);
					}
					else
					{
						memcpy(TxMessage, &(CANTx_Buffer.data[CANTx_Buffer.read_index]), 8);
					}
					CanSend(CANIDTOFC, TxMessage, 8);
					CANTx_Buffer.read_index = (CANTx_Buffer.read_index + 8)%CANTx_Buffer.size;
				}
				else
				{
					temp = CANTx_Buffer.size - CANTx_Buffer.read_index;
					if(temp < tx_len1)
					{
						memcpy(TxMessage,&(CANTx_Buffer.data[CANTx_Buffer.read_index]), temp);
						memcpy(&TxMessage[temp],&(CANTx_Buffer.data[0]) ,tx_len1-temp);
					}
					else
					{
						memcpy(TxMessage,&(CANTx_Buffer.data[CANTx_Buffer.read_index]), tx_len1);
					}
					CanSend(CANIDTOFC, TxMessage, tx_len1);
					CANTx_Buffer.read_index = (CANTx_Buffer.read_index + tx_len1)%CANTx_Buffer.size;
				}
	  	}
		}
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
	u8 msgi = msgi;

	CanRxMsg msg;
	CAN_Receive(CAN1, CAN_FIFO0, &msg);

	if(msg.IDE == CAN_Id_Standard) // 0x500+CanIDNum
	{
    if(msg.StdId == 0x500+CanIDNum)
		{
			if(Fd_InsData_State == 0)Fd_InsData_State = 1;
      for(msgi=0;msgi<msg.DLC;msgi++)
		  {
				CANRx_Buffer.data[CANRx_Buffer.write_index] = msg.Data[msgi];	
				CANRx_Buffer.write_index = (CANRx_Buffer.write_index + 1) % CANRx_Buffer.size;
		  }	
		}		
	}
}

void api_on_upgrade(void)
{
	delay_us(200);							
	RCC_RTCCLKCmd(ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);										
	RTC_WriteBackupRegister(RTC_BKP_DR1,0x1122);
	RTC_WriteBackupRegister(RTC_BKP_DR2,0x5566);
	delay_ms(10);
	NVIC_SystemReset();
}


