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
		Debug_Data.data1  = Gyro_x;//���ٶ�x��ADֵ
		Debug_Data.data2  = Gyro_y;//���ٶ�y��ADֵ
		Debug_Data.data3  = Gyro_z;//���ٶ�z��ADֵ
		
		Debug_Data.data4  = Acc_x;//���ٶ�x��ADֵ
		Debug_Data.data5  = Acc_y;//���ٶ�y��ADֵ
		Debug_Data.data6  = Acc_z;//���ٶ�z��ADֵ
		
		Debug_Data.data7  = roll_encoder;//roll�����Ƕ�
		Debug_Data.data8  = pitch_encoder;//pitch�����Ƕ�
		Debug_Data.data9  = yaw_encoder;//yaw�����Ƕ�
		
		Debug_Data.data10 = Fd_InsData.RollAngle;//�ɿ�roll(ǰ����)0.1��
		Debug_Data.data11 = Fd_InsData.PitchAngle;//�ɿ�pitch(ǰ����)0.1��
		Debug_Data.data12 = Fd_InsData.YawAngle;//�ɿ�yaw(ǰ����)0.1��
		
		Debug_Data.data13 = Fd_InsData.RollRate;//�ɿ�x����ٶ�(ǰ����)0.1��/s
		Debug_Data.data14 = Fd_InsData.PitchRate;//�ɿ�y����ٶ�(ǰ����)0.1��/s
		Debug_Data.data15 = Fd_InsData.YawRate;//�ɿ�z����ٶ�(ǰ����)0.1��/s
		
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

