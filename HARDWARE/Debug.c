#include "app.h"
#include "Motor_Control.h"
#include "math_common.h"

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
	if(Oldman_i > 7)
	{
		Oldman_i = 0;
		Debug_Data.data1  = Gyro_x;//角速度x轴AD值
		Debug_Data.data2  = Gyro_y;//角速度y轴AD值
		Debug_Data.data3  = Gyro_z;//角速度z轴AD值
		
		Debug_Data.data4  = Acc_x;//加速度x轴AD值
		Debug_Data.data5  = Acc_y;//加速度y轴AD值
		Debug_Data.data6  = Acc_z;//加速度z轴AD值
		
		Debug_Data.data7  = roll_encoder;//roll轴电机角度
		Debug_Data.data8  = pitch_encoder;//pitch轴电机角度
		Debug_Data.data9  = yaw_encoder;//yaw轴电机角度
		
		Debug_Data.data10 = Fd_InsData.RollAngle;//飞控roll(前右下)0.1度
		Debug_Data.data11 = Fd_InsData.PitchAngle;//飞控pitch(前右下)0.1度
		Debug_Data.data12 = Fd_InsData.YawAngle;//飞控yaw(前右下)0.1度
		
		Debug_Data.data13 = Fd_InsData.RollRate;//飞控x轴角速度(前右下)0.1度/s
		Debug_Data.data14 = Fd_InsData.PitchRate;//飞控y轴角速度(前右下)0.1度/s
		Debug_Data.data15 = Fd_InsData.YawRate;//飞控z轴角速度(前右下)0.1度/s
		
		Debug_Data.data16 = 0.0f;
			
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

