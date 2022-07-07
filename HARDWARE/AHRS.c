#include "app.h"
#include "AHRS.h"

/**************************************************************************************************/

Vector3 gyro_s,acc_s;
s16 Acc_x,Acc_y,Acc_z;
s16 acc_filtered_x, acc_filtered_y, acc_filtered_z;
s16 Gyro_x, Gyro_y, Gyro_z;//
float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
float GimbalAcc_x_filted,GimbalAcc_y_filted,GimbalAcc_z_filted;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float pitch_acc = 0.0f, roll_acc = 0.0f, yaw_acc = 0.0f;
float pitch_acc_filted = 0.0f, roll_acc_filted = 0.0f, yaw_acc_filted = 0.0f;
float q[4] = {1.0f,0.0f,0.0f,0.0f};

uint8_t ReadMPU6500[14];

s16 MPU6500_raw;
float  MPU6500_Temp;
int8_t Gyro_Temp = 0;

float k = 0.01f;
void Filter_LP_IIR_1(void)
{
	acc_filtered_x += k * ( Acc_x - acc_filtered_x );
	acc_filtered_y += k * ( Acc_y - acc_filtered_y );
	acc_filtered_z += k * ( Acc_z - acc_filtered_z );
}

void IMU_Update(void)
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
	
//	GimbalGyro_x = (Gyro_x/32.8f) * DEG2RAD;	
//	GimbalGyro_y = (Gyro_y/32.8f) * DEG2RAD;	
//	GimbalGyro_z = (Gyro_z/32.8f) * DEG2RAD;
	
	GimbalGyro_x = (Gyro_x/32.8f) ;	
	GimbalGyro_y = (Gyro_y/32.8f) ;	
	GimbalGyro_z = (Gyro_z/32.8f) ;
	
	//加速度滤波
	Filter_LP_IIR_1();
	GimbalAcc_x_filted = acc_filtered_x/8192.0f;
	GimbalAcc_y_filted = acc_filtered_y/8192.0f;
	GimbalAcc_z_filted = acc_filtered_z/8192.0f;
}

float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
//float sum_acc[3] = {0.0f, 0.0f, 0.0f};
float GyroOffset[3] = {0.0f, 0.0f, 0.0f};//陀螺原始AD
uint8_t calib_n = 0;
void cal_gyro_bias(void)
{
	sum_gyro[0] += Gyro_x;
	sum_gyro[1] += Gyro_y;
	sum_gyro[2] += Gyro_z;
	++calib_n;
	if( calib_n >= 200 )
	{
		calib_gyroscope = false;
		float invN = 1.0f/calib_n;
		GyroOffset[0] = sum_gyro[0]*invN;
		GyroOffset[1] = sum_gyro[1]*invN;
		GyroOffset[2] = sum_gyro[2]*invN;
	}
}

void MS_Attitude_Acconly(void)
{
	float acc_norm_filted = sqrtf(acc_filtered_x*acc_filtered_x + acc_filtered_y*acc_filtered_y + acc_filtered_z*acc_filtered_z);
	float acc_norm_x_filted,acc_norm_y_filted,acc_norm_z_filted;
	acc_norm_x_filted = acc_filtered_x / acc_norm_filted;
	acc_norm_y_filted = acc_filtered_y / acc_norm_filted;
	acc_norm_z_filted = acc_filtered_z / acc_norm_filted;
	pitch_acc_filted = - asinf(acc_norm_x_filted);
	roll_acc_filted = atan2f(acc_norm_y_filted,acc_norm_z_filted);
	
	float acc_norm = sqrtf(Acc_x*Acc_x + Acc_y*Acc_y + Acc_z*Acc_z);
	float acc_norm_x,acc_norm_y,acc_norm_z;
	acc_norm_x = Acc_x / acc_norm;
	acc_norm_y = Acc_y / acc_norm;
	acc_norm_z = Acc_z / acc_norm;
	pitch_acc = - asinf(acc_norm_x);
	roll_acc = atan2f(acc_norm_y,acc_norm_z);
}

void init_MS_Attitude(void)
{
	Read_MPU6500(ReadMPU6500);
	//前左上
	Acc_x = -((ReadMPU6500[4]<<8)|ReadMPU6500[5]);
	Acc_y = -((ReadMPU6500[0]<<8)|ReadMPU6500[1]);
	Acc_z = ((ReadMPU6500[2]<<8)|ReadMPU6500[3]);
	
	acc_filtered_x = Acc_x;
	acc_filtered_y = Acc_y;
	acc_filtered_z = Acc_z;
	
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
	
	float q_norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / q_norm;
	q[1] = q[1] / q_norm;
	q[2] = q[2] / q_norm;
	q[3] = q[3] / q_norm;
	
	roll = atan2( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
	pitch = asin( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
	yaw = atan2( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
}

float sample_time_gyro = 0.0005f;
//float delta_angle[3] = {0.0f,0.0f,0.0f};
void MS_Attitude_GyroIntegral(void)
{
	float delta_angle[3];
	delta_angle[0] = GimbalGyro_x * DEG2RAD * sample_time_gyro;
	delta_angle[1] = GimbalGyro_y * DEG2RAD * sample_time_gyro;
	delta_angle[2] = GimbalGyro_z * DEG2RAD * sample_time_gyro;
	
	float tqw=q[0];	float tqx=q[1];	float tqy=q[2];	float tqz=q[3];
	q[0] += 0.5f * ( -tqx*delta_angle[0] - tqy*delta_angle[1] - tqz*delta_angle[2] );
	q[1] += 0.5f * ( tqw*delta_angle[0] + tqy*delta_angle[2] - tqz*delta_angle[1] );
	q[2] += 0.5f * ( tqw*delta_angle[1] - tqx*delta_angle[2] + tqz*delta_angle[0] );
	q[3] += 0.5f * ( tqw*delta_angle[2] + tqx*delta_angle[1] - tqy*delta_angle[0] );
	
	float q_norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / q_norm;
	q[1] = q[1] / q_norm;
	q[2] = q[2] / q_norm;
	q[3] = q[3] / q_norm;
	
	roll = atan2( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
	pitch = asin( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
	yaw = atan2( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
}

float Kp = 0.0f;
void MS_Attitude_Mahony(void)
{
	float delta_angle[3];
	delta_angle[0] = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD * sample_time_gyro;
	delta_angle[1] = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD * sample_time_gyro;
	delta_angle[2] = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD * sample_time_gyro;
	
	float tqw=q[0];	float tqx=q[1];	float tqy=q[2];	float tqz=q[3];
	q[0] += 0.5f * ( -tqx*delta_angle[0] - tqy*delta_angle[1] - tqz*delta_angle[2] );
	q[1] += 0.5f * ( tqw*delta_angle[0] + tqy*delta_angle[2] - tqz*delta_angle[1] );
	q[2] += 0.5f * ( tqw*delta_angle[1] - tqx*delta_angle[2] + tqz*delta_angle[0] );
	q[3] += 0.5f * ( tqw*delta_angle[2] + tqx*delta_angle[1] - tqy*delta_angle[0] );
	
	float q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / q_norm;
	q[1] = q[1] / q_norm;
	q[2] = q[2] / q_norm;
	q[3] = q[3] / q_norm;
	
	float vx = 2.0f*(q[1]*q[3]-q[0]*q[2]);
	float vy = 2.0f*(q[0]*q[1]+q[2]*q[3]);
	float vz = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
	
	float acc_norm = sqrtf(acc_filtered_x*acc_filtered_x + acc_filtered_y*acc_filtered_y + acc_filtered_z*acc_filtered_z);
	float ax = acc_filtered_x / acc_norm;
	float ay = acc_filtered_y / acc_norm;
	float az = acc_filtered_z / acc_norm;
	
	float ex = ay * vz - az * vy;
	float ey = az * vx - ax * vz;
	float ez = ax * vy - ay * vx;
	
	float err_angle = sqrtf(ex*ex + ey*ey + ez*ez);
	ex = ex / err_angle;
	ey = ey / err_angle;
	ez = ez / err_angle;
	err_angle = err_angle * Kp;
	
	float half_sinerr = sinf(0.5f*err_angle);
	float half_coserr = cosf(0.5f*err_angle);
	float q_err[4] = {half_coserr,half_sinerr*ex,half_sinerr*ey,half_sinerr*ez};
	q[0] = q[0]*q_err[0] - q[1]*q_err[1] - q[2]*q_err[2] - q[3]*q_err[3];
	q[1] = q[0]*q_err[1] + q[1]*q_err[0] + q[2]*q_err[3] - q[3]*q_err[2];
	q[2] = q[0]*q_err[2] + q[2]*q_err[0] - q[1]*q_err[3] + q[3]*q_err[1];
	q[3] = q[0]*q_err[3] + q[1]*q_err[2] - q[2]*q_err[1] + q[3]*q_err[0];
	
	q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / q_norm;
	q[1] = q[1] / q_norm;
	q[2] = q[2] / q_norm;
	q[3] = q[3] / q_norm;
	
	roll = atan2f( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
	pitch = asinf( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
	yaw = atan2f( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
	
}

float Kp_acc = 1.0f;
float Ki_acc = 0.001f;
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float bias_max = 0.05f;
float gyro[3];
const float upper_accel_limit = 1.05f;
const float lower_accel_limit = 0.95f;
void MS_Attitude_Mahony_Bias(void)
{
	// Angular rate of correction
	float corr[3] = {0.0f, 0.0f, 0.0f};
//	float delta_angle[3];
	
//	float gyro[3];
	gyro[0] = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD;
	gyro[1] = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD;
	gyro[2] = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD;
	float spinRate = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
	
//	// Accelerometer correction
	float vx = 2.0f*(q[1]*q[3]-q[0]*q[2]);
	float vy = 2.0f*(q[0]*q[1]+q[2]*q[3]);
	float vz = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
	
	// fuse accel data only if its norm is close to 1 g (reduces drift).
	const float acc_norm_filted = sqrtf(acc_filtered_x*acc_filtered_x + acc_filtered_y*acc_filtered_y + acc_filtered_z*acc_filtered_z);
	const float accel_norm_sq = acc_norm_filted / 8192.0f;
//	const float acc_norm = sqrtf(Acc_x*Acc_x + Acc_y*Acc_y + Acc_z*Acc_z);
	
	if ((accel_norm_sq > lower_accel_limit) && (accel_norm_sq < upper_accel_limit))
	{
		float ax = acc_filtered_x / acc_norm_filted;
		float ay = acc_filtered_y / acc_norm_filted;
		float az = acc_filtered_z / acc_norm_filted;
	
		float ex = ay * vz - az * vy;
		float ey = az * vx - ax * vz;
		float ez = ax * vy - ay * vx;
		
		corr[0] += ex * Kp_acc;
		corr[1] += ey * Kp_acc;
		corr[2] += ez * Kp_acc;
	}
	
	// Gyro bias estimation
	if (spinRate < 0.175f)
	{
		gyro_bias[0] += corr[0] * Ki_acc;
		gyro_bias[1] += corr[1] * Ki_acc;
		gyro_bias[2] += corr[2] * Ki_acc;
//	
//	for (int i = 0; i < 3; i++) 
//	{
//		if( gyro_bias[i] > bias_max)
//		{
//			gyro_bias[i] = bias_max;
//		}
//		
//		if( gyro_bias[i] < -bias_max)
//		{
//			gyro_bias[i] = -bias_max;
//		}
//	}
		
	}
//	
	corr[0] += gyro_bias[0];
	corr[1] += gyro_bias[1];
	corr[2] += gyro_bias[2];
	
	// Feed forward gyro
	corr[0] += gyro[0];
	corr[1] += gyro[1];
	corr[2] += gyro[2];
	
	corr[0] *= sample_time_gyro;
	corr[1] *= sample_time_gyro;
	corr[2] *= sample_time_gyro;
	
//	delta_angle[0] = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD * sample_time_gyro;
//	delta_angle[1] = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD * sample_time_gyro;
//	delta_angle[2] = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD * sample_time_gyro;
	
	// Apply correction to state
	float tqw=q[0];	float tqx=q[1];	float tqy=q[2];	float tqz=q[3];
	q[0] += 0.5f * ( -tqx*corr[0] - tqy*corr[1] - tqz*corr[2] );
	q[1] += 0.5f * ( tqw*corr[0] + tqy*corr[2] - tqz*corr[1] );
	q[2] += 0.5f * ( tqw*corr[1] - tqx*corr[2] + tqz*corr[0] );
	q[3] += 0.5f * ( tqw*corr[2] + tqx*corr[1] - tqy*corr[0] );
	
	// Normalize quaternion
	float q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / q_norm;
	q[1] = q[1] / q_norm;
	q[2] = q[2] / q_norm;
	q[3] = q[3] / q_norm;
	
	roll = atan2f( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
	pitch = asinf( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
	yaw = atan2f( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
}
