#include "app.h"

#define MPU6500_OFF  GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define MPU6500_ON  GPIO_ResetBits(GPIOA,GPIO_Pin_15)

void SPI3_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE );//PORTB时钟使能 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE );//SPI2时钟使能 	
	
	SPI_I2S_DeInit(SPI3);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_6);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_15);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//主机模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8位数据
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_High=模式3，时钟空闲为高 //SPI_CPOL_Low=模式0，时钟空闲为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//SPI_CPHA_2Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Soft;//SPI_NSS_Hard
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_2=32M;//SPI_BaudRatePrescaler_4=18MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据从高位开始发送
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_NSSInternalSoftwareConfig(SPI3,SPI_NSSInternalSoft_Set);
  SPI_RxFIFOThresholdConfig(SPI3,SPI_RxFIFOThreshold_QF);
	SPI_Cmd(SPI3, ENABLE); //使能SPI外设
	delay_ms(1);
	MPU6500_Init();	
	delay_ms(500);
} 


/***************************************************************/
static u8 SPI3_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;		

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
	{
		retry++;
		if(retry>100)return 0;
	}			  
	SPI_SendData8(SPI3, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
	{
		retry++;
		if(retry>100)return 0;
	}	  						    
	return SPI_ReceiveData8(SPI3); //接收数据					    
}

u8 MPU6500_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	MPU6500_ON;
	status = SPI3_ReadWriteByte(reg);
	SPI3_ReadWriteByte(value);
	MPU6500_OFF;
	return(status);
}

u8 MPU6500_Read_Reg(u8 reg)
{
	u8 reg_val;
	MPU6500_ON;
	SPI3_ReadWriteByte(reg|0x80);
	reg_val=SPI3_ReadWriteByte(0xff);
	MPU6500_OFF;
	return(reg_val);
}

u8 WHO_AM_I = 0;

void MPU6500_Init(void)
{
	while(WHO_AM_I != 0x70)
	{
	  WHO_AM_I = MPU6500_Read_Reg(MPU6500_WHO_AM_I);
		delay_us(500);
		R_LED_Status(4);
	}
	R_LED_Status(0);
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_PWR_MGMT_1,0X80);   		//电源管理,复位MPU6500
	delay_ms(100);
	MPU6500_Write_Reg(MPU6500_PWR_MGMT_1,0X03);   		//选择时钟源,Z轴陀螺作为参考
	delay_ms(500);
	MPU6500_Write_Reg(MPU6500_SIGNAL_PATH_RESET,0X07);//陀螺仪、加速度计、温度计复位
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_CONFIG,0X0);						//设置0x27
	delay_ms(10);
	//250d/s  131LSB/d 
	//500d/s  65.5LSB/d 
	//1000d/s 32.8LSB/d 
	//2000d/s 16.4LSB/d 
	//0x00 250d/s 0x08 500d/s 0x10 1000d/s 0x11 2000d/s 		
	MPU6500_Write_Reg(MPU6500_GYRO_CONFIG,0x10);  		//陀螺仪测量范围20689 - 0x00=250,0x08=500,0x10=1000,0x18正负2000 0x12
	delay_ms(10);
	WHO_AM_I = MPU6500_Read_Reg(MPU6500_GYRO_CONFIG);
	delay_ms(10);
	//2g  16384LSB/g
	//4g  8192LSB/g
	//8g  4096LSB/g
	//16g 2048LSB/g
	//0x00 2g 0x08 4g 0x10 8g 0x11 16g 
	MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG,0x08); 		//加速度计测量范围 0x00=2G,0x08=4G,0x10=8G,0x18=16G 
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG_2,0x08);
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_INT_PIN_CFG,0x50);
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_INT_ENABLE,0x01);
	delay_ms(10);
}
 

void Read_MPU6500(u8 *buf)
{
	u8 i_MPU6500;
	MPU6500_ON;
	SPI3_ReadWriteByte(MPU6500_ACCEL_XOUT_H|0x80);
	for(i_MPU6500 = 0;i_MPU6500 < 14;i_MPU6500++)
	{
		buf[i_MPU6500] = SPI3_ReadWriteByte(0xff);
	}
	MPU6500_OFF;
}

