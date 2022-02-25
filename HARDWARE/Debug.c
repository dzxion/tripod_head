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

void Oscilloscope(void)
{
	uint8_t i;
  
	Oldman_i++;
	if(Oldman_i > 10)
	{
		Oldman_i = 0;
		Debug_Data.data1  = Roll_Speed_PID.PID_Out;
		Debug_Data.data2  = Pitch_Speed_PID.PID_Out;		
		Debug_Data.data3  = Yaw_Speed_PID.PID_Out;
		Debug_Data.data4  = MPU6500_Temp;
		Debug_Data.data5  = Gyro_x;
		
		Debug_Data.data6  = Gyro_y;
		Debug_Data.data7  = Gyro_z;
		Debug_Data.data8  = roll_encoder * RAD2DEG;
		Debug_Data.data9  = pitch_encoder * RAD2DEG;
		Debug_Data.data10 = Get_Encoder.Angle_Y;
		
		Debug_Data.data11 = roll * RAD2DEG;
		Debug_Data.data12 = pitch * RAD2DEG;
		Debug_Data.data13 = yaw * RAD2DEG;
		Debug_Data.data14 = roll_acc * RAD2DEG;
		Debug_Data.data15 = pitch_acc * RAD2DEG;
		Debug_Data.data16 = yaw_acc * RAD2DEG;
			
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

