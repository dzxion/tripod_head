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
extern u8 tim4_count;

void Oscilloscope(void)
{
	uint8_t i;
  
	Oldman_i++;
	if(Oldman_i > 7)
	{
		Oldman_i = 0;
		Debug_Data.data1  = /*gyro.x;*/roll_acc_filted*RAD2DEG;/*target_quat.qw;*/
		Debug_Data.data2  = /*gyro.y;*/pitch_acc_filted*RAD2DEG;/*target_quat.qx;*/
		Debug_Data.data3  = /*gyro.z;*/euler_Est.x*RAD2DEG;/*target_quat.qy;*/
		
		Debug_Data.data4  = /*acc.x;*/euler_Est.y*RAD2DEG;/*target_quat.qz;*/
		Debug_Data.data5  = /*acc.y;*/euler_Est.z*RAD2DEG;
		Debug_Data.data6  = /*acc.z;*/bias.x*RAD2DEG;
		
		Debug_Data.data7  = /*Encoder.x*RAD2DEG;*/debug_buf[0];
		Debug_Data.data8  = /*Encoder.y*RAD2DEG;*/debug_buf[1];
		Debug_Data.data9  = /*Encoder.z*RAD2DEG;*/debug_buf[2];
		
		Debug_Data.data10 = Angle_Vehicle.x*RAD2DEG;//debug_buf[3];
		Debug_Data.data11 = Angle_Vehicle.y*RAD2DEG;//debug_buf[4];
		Debug_Data.data12 = Angle_Vehicle.z*RAD2DEG;//debug_buf[5];
		
		Debug_Data.data13 = /*Fd_InsData.RollRate;*/debug_buf[9];
		Debug_Data.data14 = /*Fd_InsData.PitchRate;*/debug_buf[10];
		Debug_Data.data15 = /*Fd_InsData.YawRate;*/debug_buf[11];
		
		Debug_Data.data16 = /*0.0f;*/Get_Motor.rev1;
		
//		tim4_count = 0;
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

