#include "app.h"

/**************************************************************************************************/
s16 Acc_x,Acc_y,Acc_z;
s16 Gyro_x, Gyro_y, Gyro_z;//
float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

uint8_t ReadMPU6500[14];

s16 MPU6500_raw;
float  MPU6500_Temp;
int8_t Gyro_Temp = 0;

void Conversion(void)
{
	Read_MPU6500(ReadMPU6500);
	//右上后
//	Acc_x = ((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
//	Acc_y = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
//	Acc_z = ((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	
	//前左上
	Acc_x = -((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	Acc_y = -((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
	Acc_z = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
	
	MPU6500_raw = ((ReadMPU6500[6]<<8)|ReadMPU6500[7]) - 21;
	MPU6500_Temp = 20.0f + (float)(MPU6500_raw/333.87f);	
	Gyro_Temp = (int8_t)MPU6500_Temp;
	//右后上
//	Gyro_x = ((ReadMPU6500[8]<<8)|ReadMPU6500[9]);//X
//	Gyro_y = ((ReadMPU6500[12]<<8)|ReadMPU6500[13]);//Y
//	Gyro_z = ((ReadMPU6500[10]<<8)|ReadMPU6500[11]);//Z
	
	//前左上
	Gyro_x = -((ReadMPU6500[12]<<8)|ReadMPU6500[13]);//X
	Gyro_y = -((ReadMPU6500[8]<<8)|ReadMPU6500[9]);//Y
	Gyro_z = ((ReadMPU6500[10]<<8)|ReadMPU6500[11]);//Z
	
	GimbalGyro_x = Gyro_x/32.8f;	
	GimbalGyro_y = Gyro_y/32.8f;	
	GimbalGyro_z = Gyro_z/32.8f;	
}

void MS_Attitude_Acconly(void)
{
	float acc_norm = sqrt(Acc_x*Acc_x + Acc_y*Acc_y + Acc_z*Acc_z);
	float acc_norm_x,acc_norm_y,acc_norm_z;
	acc_norm_x = Acc_x / acc_norm;
	acc_norm_y = Acc_y / acc_norm;
	acc_norm_z = Acc_z / acc_norm;
	pitch = - asin(acc_norm_x) * RAD2DEG;
	roll = atan2(acc_norm_y,acc_norm_z) * RAD2DEG;
}

void MS_Attitude_GyroIntegral(void)
{
	
}

void MS_Attitude(void)
{
	
}

