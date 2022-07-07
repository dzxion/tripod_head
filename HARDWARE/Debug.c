#include "app.h"

u8 get_decode_data[]=
{	
	0x23,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00
};

u8 Oldman_i = 0;

_debug_data Debug_Data;
float debug_buf[16];

void Oscilloscope(void)
{
	uint8_t i;
  
	Oldman_i++;
	if(Oldman_i > 10)
	{
		Oldman_i = 0;
		Debug_Data.data1  = roll * RAD2DEG;//x轴目标机体角速度
		Debug_Data.data2  = pitch * RAD2DEG;//y轴目标机体角速度
		Debug_Data.data3  = yaw * RAD2DEG;//z轴目标机体角速度
		Debug_Data.data4  = roll_encoder * RAD2DEG;//w温度
		Debug_Data.data5  = pitch_encoder * RAD2DEG;//x轴陀螺角速度
		
		Debug_Data.data6  = 0.0f;//y轴陀螺角速度
		Debug_Data.data7  = 0.0f;//z轴陀螺角速度
		Debug_Data.data8  = 0.0f;//编码器算的roll
		Debug_Data.data9  = 0.0f;//编码器算的pitch
		Debug_Data.data10 = debug_buf[0];//编码器算的yaw
		
		Debug_Data.data11 = debug_buf[1];//融合算法算的roll
		Debug_Data.data12 = debug_buf[2];//融合算法算的pitch
		Debug_Data.data13 = debug_buf[3];//融合算法算的yaw
		Debug_Data.data14 = debug_buf[4];//加速度计算的roll
		Debug_Data.data15 = debug_buf[5];//加速度计算的pitch
		Debug_Data.data16 = 0.0F;//加速度计算的yaw
			
	  get_decode_data[0] = 0x23;
		memcpy(get_decode_data+1,&Debug_Data,sizeof(Debug_Data));
		
		get_decode_data[65]=0;
		for(i=1;i<65;i++)
		{	  				    
			get_decode_data[65] += get_decode_data[i];
		}	
		UART1_SendDataDMA(get_decode_data,66);
	}
}

