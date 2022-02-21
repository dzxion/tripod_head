#include "app.h"

/**************************************************************************************************/
s16 Acc_x,Acc_y,Acc_z;
s16 Gyro_x, Gyro_y, Gyro_z;//
float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float pitch_acc = 0.0f, roll_acc = 0.0f, yaw_acc = 0.0f;
float q[4] = {1,0,0,0};

uint8_t ReadMPU6500[14];

s16 MPU6500_raw;
float  MPU6500_Temp;
int8_t Gyro_Temp = 0;

void Conversion(void)
{
	Read_MPU6500(ReadMPU6500);
	//���Ϻ�
//	Acc_x = ((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
//	Acc_y = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
//	Acc_z = ((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	
	//ǰ����
	Acc_x = -((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	Acc_y = -((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
	Acc_z = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
	
	MPU6500_raw = ((ReadMPU6500[6]<<8)|ReadMPU6500[7]) - 21;
	MPU6500_Temp = 20.0f + (float)(MPU6500_raw/333.87f);	
	Gyro_Temp = (int8_t)MPU6500_Temp;
	//�Һ���
//	Gyro_x = ((ReadMPU6500[8]<<8)|ReadMPU6500[9]);//X
//	Gyro_y = ((ReadMPU6500[12]<<8)|ReadMPU6500[13]);//Y
//	Gyro_z = ((ReadMPU6500[10]<<8)|ReadMPU6500[11]);//Z
	
	//ǰ����
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
	pitch_acc = - asin(acc_norm_x);
	roll_acc = atan2(acc_norm_y,acc_norm_z);
}

void init_MS_Attitude(void)
{
	Read_MPU6500(ReadMPU6500);
	//ǰ����
	Acc_x = -((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	Acc_y = -((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
	Acc_z = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
	
	float acc_norm = sqrt(Acc_x*Acc_x + Acc_y*Acc_y + Acc_z*Acc_z);
	float acc_norm_x,acc_norm_y,acc_norm_z;
	acc_norm_x = Acc_x / acc_norm;
	acc_norm_y = Acc_y / acc_norm;
	acc_norm_z = Acc_z / acc_norm;
	pitch = - asin(acc_norm_x);
	roll = atan2(acc_norm_y,acc_norm_z);
	
	float half_sinR, half_cosR;
	float half_sinP, half_cosP;
	float half_sinY, half_cosY;
	half_sinR = sin(0.5f*roll); half_cosR = cos(0.5f*roll);
	half_sinP = sin(0.5f*pitch); half_cosP = cos(0.5f*pitch);
	half_sinY = sin(0.5f*yaw); half_cosY = cos(0.5f*yaw);
	
	q[0] = half_cosR*half_cosP*half_cosY + half_sinR*half_sinP*half_sinY;
	q[1] = half_sinR*half_cosP*half_cosY - half_cosR*half_sinP*half_sinY;
	q[2] = half_cosR*half_sinP*half_cosY + half_sinR*half_cosP*half_sinY;
	q[3] = half_cosR*half_cosP*half_sinY - half_sinR*half_sinP*half_cosY;
	
	roll = atan2( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
	pitch = asin( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
	yaw = atan2( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
}

void MS_Attitude_GyroIntegral(void)
{
	
}

void MS_Attitude(void)
{
	
}

