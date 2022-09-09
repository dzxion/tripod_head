#include "app.h"
#include "AHRS.h"
#include <float.h>
#include "math_common.h"

/**************************************************************************************************/

bool AHRS_OK = false;
vector3 gyro = {0.0f,0.0f,0.0f};//陀螺角速度（rad/s）
vector3 acc = {0.0f,0.0f,0.0f};//加速度（g）
vector3 Encoder = {0.0f,0.0f,0.0f};//编码器角度（rad）
vector3 Angle_Vehicle = {0.0f,0.0f,0.0f};//飞控角度（rad）
vector3 acc_filted = {0.0f,0.0f,0.0f};//加速度滤波（g）
vector3 euler_Est = {0.0f,0.0f,0.0f};//估计的欧拉角
float roll_acc_filted = 0.0f;//使用加速度计计算的roll（rad） 
float pitch_acc_filted = 0.0f;//使用加速度计计算的pitch（rad）
float yaw_acc_filted = 0.0f;
Quaternion qEstC = {1.0f,0.0f,0.0f,0.0f};//基于加速度互补滤波估计的姿态四元数
Quaternion qEstC_fc = {1.0f,0.0f,0.0f,0.0f};//基于飞控姿态互补滤波估计的姿态四元数
Quaternion qEst = {1.0f,0.0f,0.0f,0.0f};//估计的姿态四元数
vector3 biasC = {0.0f,0.0f,0.0f};//基于加速度互补滤波估计的陀螺零偏（rad/s）
vector3 biasC_fc = {0.0f,0.0f,0.0f};//基于飞控姿态互补滤波估计的陀螺零偏（rad/s）
vector3 bias = {0.0f,0.0f,0.0f};//估计的陀螺零偏（rad/s）
const float fs = 2000.0f;
//const float dt = 0.0005f;
s16 Acc_x,Acc_y,Acc_z;
s16 acc_filtered_x, acc_filtered_y, acc_filtered_z;
s16 Gyro_x, Gyro_y, Gyro_z;//
float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
float GimbalAcc_x_filted,GimbalAcc_y_filted,GimbalAcc_z_filted;
float rollEstC = 0.0f, pitchEstC = 0.0f, yawEstC = 0.0f;
float rollEstC_fc = 0.0f,pitchEstC_fc = 0.0f,yawEstC_fc = 0.0f;
float rollEst = 0.0f, pitchEst = 0.0f, yawEst = 0.0f;
float pitch_acc = 0.0f, roll_acc = 0.0f, yaw_acc = 0.0f;
float q[4] = {1.0f,0.0f,0.0f,0.0f};

uint8_t ReadMPU6500[14];

s16 MPU6500_raw;
float  MPU6500_Temp;
int8_t Gyro_Temp = 0;

/****************************************************************************************
*原  型：void Get_Vehicle_Attitude(vector3* Attitude_Vehicle,Fd_InsData_t Attitude_Data)
*功  能：获取载体姿态
*输  入：*Attitude_Vehicle ：姿态结构体，单位rad
*输  出：无
*****************************************************************************************/
void Get_Vehicle_Attitude(vector3* Attitude_Vehicle,Fd_InsData_t Attitude_Data)
{
	Attitude_Vehicle->x = Attitude_Data.RollAngle*0.1f*DEG2RAD;
	Attitude_Vehicle->y = -Attitude_Data.PitchAngle*0.1f*DEG2RAD;
	Attitude_Vehicle->z = -Attitude_Data.YawAngle*0.1f*DEG2RAD;
}

/****************************************************************************************
*原  型：void Filter_LP_IIR_1(vector3* output, vector3 input)
*功  能：一阶低通滤波
*输  入：*output :滤波器输出
*输  入：input :滤波器输入
*输  出：无
*****************************************************************************************/
float k_lp = 0.1f;
void Filter_LP_IIR_1(vector3* output, vector3 input)
{
	output->x += k_lp * ( input.x - output->x );
	output->y += k_lp * ( input.y - output->y );
	output->z += k_lp * ( input.z - output->z );
}

/****************************************************************************************
*原  型：void IMU_Update(vector3* gyro, vector3* acc)
*功  能：IMU数据更新
*输  入：*gyro :gyro结构体变量，单位rad/s
*输  入：*acc :acc结构体变量，单位g
*输  出：无
*****************************************************************************************/
void IMU_Update(vector3* gyro, vector3* acc)
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
	
	gyro->x = (Gyro_x/32.8f) * DEG2RAD;	
	gyro->y = (Gyro_y/32.8f) * DEG2RAD;	
	gyro->z = (Gyro_z/32.8f) * DEG2RAD;
	
	acc->x = Acc_x/8192.0f;
	acc->y = Acc_y/8192.0f;
	acc->z = Acc_z/8192.0f;
	
	//加速度滤波
//	Filter_LP_IIR_1();
//	GimbalAcc_x_filted = acc_filtered_x/8192.0f;
//	GimbalAcc_y_filted = acc_filtered_y/8192.0f;
//	GimbalAcc_z_filted = acc_filtered_z/8192.0f;
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

/****************************************************************************************
*原  型：void MS_Attitude_Acconly(vector3 acc, float* roll, float* pitch)
*功  能：使用加速度计算角度
*输  入：acc :加速度，单位g
*输  入：*roll :横滚角，单位rad
*输  入：*pitch :俯仰角，单位rad
*输  出：无
*****************************************************************************************/
void MS_Attitude_Acconly(vector3 acc, float* roll, float* pitch)
{
	float acc_norm = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
	acc.x = acc.x / acc_norm;
	acc.y = acc.y / acc_norm;
	acc.z = acc.z / acc_norm;
	*pitch = - asinf(acc.x);
	*roll = atan2f(acc.y,acc.z);
}

Quaternion Attitude_quat;//相机姿态四元数
vector3 correction_filted;//相机姿态修正量

/****************************************************************************************
*原  型：void init_MS_Attitude(void)
*功  能：姿态初始化
*输  入：无
*输  出：无
*****************************************************************************************/
void init_MS_Attitude(Quaternion* q)
{
	float roll = 0.0f,pitch = 0.0f,yaw = 0.0f;
	MS_Attitude_Acconly(acc, &roll, &pitch);
	yaw = Angle_Vehicle.z;
	Euler2Quat( q, roll, pitch, yaw );
	
	AHRS_OK = true;
}

float sample_time_gyro = 0.0005f;
//float delta_angle[3] = {0.0f,0.0f,0.0f};
void MS_Attitude_GyroIntegral(Quaternion* q, vector3 gyro, vector3 bias, float dt)
{	
	vector3 delta_angle;
	delta_angle.x = (gyro.x + bias.x) * dt;
	delta_angle.y = (gyro.y + bias.y) * dt;
	delta_angle.z = (gyro.z + bias.z) * dt;
	integral( q, delta_angle );
//	Quat2Euler( &euler_Est, *q );
}

float Kp = 0.0f;
//mahony不带bias解算
//void MS_Attitude_Mahony(void)
//{
//	float delta_angle[3];
//	delta_angle[0] = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD * sample_time_gyro;
//	delta_angle[1] = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD * sample_time_gyro;
//	delta_angle[2] = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD * sample_time_gyro;
//	
//	float tqw=q[0];	float tqx=q[1];	float tqy=q[2];	float tqz=q[3];
//	q[0] += 0.5f * ( -tqx*delta_angle[0] - tqy*delta_angle[1] - tqz*delta_angle[2] );
//	q[1] += 0.5f * ( tqw*delta_angle[0] + tqy*delta_angle[2] - tqz*delta_angle[1] );
//	q[2] += 0.5f * ( tqw*delta_angle[1] - tqx*delta_angle[2] + tqz*delta_angle[0] );
//	q[3] += 0.5f * ( tqw*delta_angle[2] + tqx*delta_angle[1] - tqy*delta_angle[0] );
//	
//	float q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
//	q[0] = q[0] / q_norm;
//	q[1] = q[1] / q_norm;
//	q[2] = q[2] / q_norm;
//	q[3] = q[3] / q_norm;
//	
//	float vx = 2.0f*(q[1]*q[3]-q[0]*q[2]);
//	float vy = 2.0f*(q[0]*q[1]+q[2]*q[3]);
//	float vz = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
//	
//	float acc_norm = sqrtf(acc_filtered_x*acc_filtered_x + acc_filtered_y*acc_filtered_y + acc_filtered_z*acc_filtered_z);
//	float ax = acc_filtered_x / acc_norm;
//	float ay = acc_filtered_y / acc_norm;
//	float az = acc_filtered_z / acc_norm;
//	
//	float ex = ay * vz - az * vy;
//	float ey = az * vx - ax * vz;
//	float ez = ax * vy - ay * vx;
//	
//	float err_angle = sqrtf(ex*ex + ey*ey + ez*ez);
//	ex = ex / err_angle;
//	ey = ey / err_angle;
//	ez = ez / err_angle;
//	err_angle = err_angle * Kp;
//	
//	float half_sinerr = sinf(0.5f*err_angle);
//	float half_coserr = cosf(0.5f*err_angle);
//	float q_err[4] = {half_coserr,half_sinerr*ex,half_sinerr*ey,half_sinerr*ez};
//	q[0] = q[0]*q_err[0] - q[1]*q_err[1] - q[2]*q_err[2] - q[3]*q_err[3];
//	q[1] = q[0]*q_err[1] + q[1]*q_err[0] + q[2]*q_err[3] - q[3]*q_err[2];
//	q[2] = q[0]*q_err[2] + q[2]*q_err[0] - q[1]*q_err[3] + q[3]*q_err[1];
//	q[3] = q[0]*q_err[3] + q[1]*q_err[2] - q[2]*q_err[1] + q[3]*q_err[0];
//	
//	q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
//	q[0] = q[0] / q_norm;
//	q[1] = q[1] / q_norm;
//	q[2] = q[2] / q_norm;
//	q[3] = q[3] / q_norm;
//	
//	roll = atan2f( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
//	pitch = asinf( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
//	yaw = atan2f( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
//	
//}

float Kp_acc = 1.0f,Kp_yaw = 10.0f;
float Ki_acc = 0.001f;
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float bias_max = 3.0f*DEG2RAD;
//float gyro[3];
const float upper_accel_limit = 1.05f;
const float lower_accel_limit = 0.95f;
bool use_yaw_encoder = true;

/****************************************************************************************
*原  型：void MS_Attitude_Mahony_Bias(Quaternion* q,vector3* bias,vector3 gyro,vector3 acc)
*功  能：基于加速度的姿态估计（互补滤波）带零偏
*输  入：*q 估计的姿态四元数
*输  入：*bias 估计的陀螺零偏 rad/s
*输  入：gyro 陀螺角速度 rad/s
*输  入：acc 加速度 g
*输  出：无
*****************************************************************************************/
void MS_Attitude_CF_Bias(Quaternion* q,vector3* bias,vector3 gyro,vector3 acc)
{
	// Angular rate of correction
	vector3 corr = {0.0f, 0.0f, 0.0f};
//	float delta_angle[3];
	
//	float gyro[3];	
//	gyro.x = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD;
//	gyro.y = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD;
//	gyro.z = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD;
//	float spinRate = sqrtf(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z);
	
	vector3 vec = {0.0f, 0.0f, 0.0f};
	vec.x = 2.0f*(q->qx*q->qz - q->qw*q->qy);
	vec.y = 2.0f*(q->qw*q->qx + q->qy*q->qz);
	vec.z = q->qw*q->qw - q->qx*q->qx - q->qy*q->qy + q->qz*q->qz;
	
//	if (use_yaw_encoder == true && pitch * RAD2DEG < 45)
//	{
//		float angle_error = Kp_yaw * (yaw_by_encoder - yaw);
//		float ex = angle_error * vx;
//		float ey = angle_error * vy;
//		float ez = angle_error * vz;
//		corr[0] += ex;
//		corr[1] += ey;
//		corr[2] += ez;
//	}
	
//	// Accelerometer correction
	
	// fuse accel data only if its norm is close to 1 g (reduces drift).
    float acc_norm = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
//	const float accel_norm_sq = acc_norm_filted / 8192.0f;
//	const float acc_norm = sqrtf(Acc_x*Acc_x + Acc_y*Acc_y + Acc_z*Acc_z);
	
	acc.x = acc.x / acc_norm;
	acc.y = acc.y / acc_norm;
	acc.z = acc.z / acc_norm;
	
	vector3 e = {0.0f, 0.0f, 0.0f};
	e.x = acc.y*vec.z - acc.z*vec.y;
	e.y = acc.z*vec.x - acc.x*vec.z;
	e.z = acc.x*vec.y - acc.y*vec.x;
	
	corr.x = Kp_acc * e.x;
	corr.y = Kp_acc * e.y;
	corr.z = Kp_acc * e.z;
//	if ((accel_norm_sq > lower_accel_limit) && (accel_norm_sq < upper_accel_limit))
//	{
//		float ax = acc_filtered_x / acc_norm_filted;
//		float ay = acc_filtered_y / acc_norm_filted;
//		float az = acc_filtered_z / acc_norm_filted;
//	
//		float ex = ay * vz - az * vy;
//		float ey = az * vx - ax * vz;
//		float ez = ax * vy - ay * vx;
//		
//		corr[0] += ex * Kp_acc;
//		corr[1] += ey * Kp_acc;
//		corr[2] += ez * Kp_acc;
//	}
	
	// Gyro bias estimation
//	if (spinRate < 0.175f)
//	{
		bias->x += corr.x * Ki_acc;
		bias->y += corr.y * Ki_acc;
		bias->z += corr.z * Ki_acc;
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
	//Integral limiter
	if( bias->x > bias_max)
	{
		bias->x = bias_max;
	}		
	if( bias->x < -bias_max)
	{
		bias->x = -bias_max;
	}
	
	if( bias->y > bias_max)
	{
		bias->y = bias_max;
	}
	if( bias->y < -bias_max)
	{
		bias->y = -bias_max;
	}
	
	if( bias->z > bias_max)
	{
		bias->z = bias_max;
	}
	if( bias->z < -bias_max)
	{
		bias->z = -bias_max;
	}
	
	bias->z = 0.0f;
		
//	}
//	
//	corr[0] += gyro_bias[0];
//	corr[1] += gyro_bias[1];
//	corr[2] += gyro_bias[2];
//	
//	// Feed forward gyro
//	corr[0] += gyro.x;
//	corr[1] += gyro.y;
//	corr[2] += gyro.z;
//	
//	corr[0] *= sample_time_gyro;
//	corr[1] *= sample_time_gyro;
//	corr[2] *= sample_time_gyro;
//	
////	delta_angle[0] = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD * sample_time_gyro;
////	delta_angle[1] = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD * sample_time_gyro;
////	delta_angle[2] = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD * sample_time_gyro;
//	
//	// Apply correction to state
//	float tqw=q[0];	float tqx=q[1];	float tqy=q[2];	float tqz=q[3];
//	q[0] += 0.5f * ( -tqx*corr[0] - tqy*corr[1] - tqz*corr[2] );
//	q[1] += 0.5f * ( tqw*corr[0] + tqy*corr[2] - tqz*corr[1] );
//	q[2] += 0.5f * ( tqw*corr[1] - tqx*corr[2] + tqz*corr[0] );
//	q[3] += 0.5f * ( tqw*corr[2] + tqx*corr[1] - tqy*corr[0] );
//	
//	// Normalize quaternion
//	float q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
//	q[0] = q[0] / q_norm;
//	q[1] = q[1] / q_norm;
//	q[2] = q[2] / q_norm;
//	q[3] = q[3] / q_norm;
	
	vector3 delta_angle;
	delta_angle.x = (gyro.x + bias->x + corr.x) * dt;
	delta_angle.y = (gyro.y + bias->y + corr.y) * dt;
	delta_angle.z = (gyro.z + bias->z + corr.z) * dt;
	
	integral( q, delta_angle );
	
//	//旋转矢量转四元数尝试
//	float rotation_vector[3];
//	float theta = 2.0f* acosf( q[0] );
//	if(theta > PI)
//		theta -= 2.0f*PI;
//	float sin_half_theta = sqrtf( 1.0f - q[0]*q[0] );
////	float scale = theta / sin_half_theta;
//	float scale;
//	if (fabsf(sin_half_theta) < FLT_EPSILON)
//		scale = 2.0f;
//	else
//		scale = theta / sin_half_theta;
//	
//	rotation_vector[0] = q[1] * scale * RAD2DEG;
//	rotation_vector[1] = q[2] * scale * RAD2DEG;
//	rotation_vector[2] = q[3] * scale * RAD2DEG;
	
//	Quat2Euler( &euler_Est, *q );
}

void Get_Encoder_Quat( Quaternion* q, float roll, float pitch, float yaw )
{
	float half_sinR, half_cosR;
	float half_sinP, half_cosP;
	float half_sinY, half_cosY;
	half_sinR = sinf(0.5f*roll); half_cosR = cosf(0.5f*roll);
	half_sinP = sinf(0.5f*pitch); half_cosP = cosf(0.5f*pitch);
	half_sinY = sinf(0.5f*yaw); half_cosY = cosf(0.5f*yaw);
	q->qw = half_cosR*half_cosP*half_cosY - half_sinR*half_sinP*half_sinY;
	q->qx = half_sinR*half_cosP*half_cosY - half_cosR*half_sinP*half_sinY;
	q->qy = half_cosR*half_sinP*half_cosY + half_sinR*half_cosP*half_sinY;
	q->qz = half_cosR*half_cosP*half_sinY + half_sinR*half_sinP*half_cosY;
	
	normalize(q);
}

//使用飞控姿态修正
float Kp_fc = 0.1f;
float Ki_fc = 0.1f;
//float K_lp = 0.01f;
/****************************************************************************************
*原  型：void MS_Attitude_Mahony_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder)
*功  能：基于飞控姿态的姿态估计（互补滤波）带零偏
*输  入：*q 估计的姿态四元数
*输  入：*bias 估计的陀螺零偏 rad/s
*输  入：gyro 陀螺角速度 rad/s
*输  入：angle_vehicle 姿态角度 rad
*输  入：encoder 编码器角度 rad
*输  出：无
*****************************************************************************************/
void MS_Attitude_CF_Bias_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder)
{
//	vector3 correction;
//	correction.x = 0.0f;
//	correction.y = 0.0f;
//	correction.z = 0.0f;
	
	//陀螺积分
	vector3 delta_angle;
	delta_angle.x = (gyro.x + bias->x) * dt;
	delta_angle.y = (gyro.y + bias->y) * dt;
	delta_angle.z = (gyro.z + bias->z) * dt;
	integral( q, delta_angle );
	
	if(ahrs_count >= 8)//8ms使用飞控修正 
	{
		ahrs_count = 0;
		//获取飞控姿态并转换到前左上坐标系
//		Quaternion quat_fc;
//		float roll = Fd_InsData.RollAngle * 0.1f * DEG2RAD;
//		float pitch = -Fd_InsData.PitchAngle * 0.1f * DEG2RAD;
////		float yaw = -Fd_InsData.YawAngle * 0.1f * DEG2RAD;
//		float yaw = 0.0f;
	
		//计算飞控姿态四元数
		Quaternion quat_fc;
		Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, angle_vehicle.z );
	
		//转换欧拉角
//		float roll_fc,pitch_fc,yaw_fc;
//		Quat2Euler( &roll_fc, &pitch_fc, &yaw_fc, fc_quat );
	
		//获取编码器角度并计算四元数
		Quaternion quat_encoder;
		Get_Encoder_Quat( &quat_encoder, encoder.x, encoder.y, encoder.z);
	
		//结合飞控姿态计算相机姿态
		Quaternion quat_measure;
		Quat_Prod( &quat_measure, quat_fc, quat_encoder );
//		float roll_cam,pitch_cam,yaw_cam;
//		Quat2Euler( &roll_cam, &pitch_cam, &yaw_cam, current_quat );

		//求误差四元数
		Quaternion current_quat_conj = *q;
		conjugate(&current_quat_conj);
		Quaternion quat_error;
		Quat_Prod( &quat_error, current_quat_conj, quat_measure );
		vector3 rotation_error;
		Quat2Axis( &rotation_error, quat_error );
		
		//求相机姿态修正量
		vector3 corr;
		corr.x = Kp_fc * rotation_error.x;
		corr.y = Kp_fc * rotation_error.y;
		corr.z = Kp_fc * rotation_error.z;
		
		//对修正量进行低通滤波
//		correction_filted.x += K_lp * ( new_data.x - correction_filted.x );
//		correction_filted.y += K_lp * ( new_data.y - correction_filted.y );
//		correction_filted.z += K_lp * ( new_data.z - correction_filted.z );
//		
//		correction.x = correction_filted.x;
//		correction.y = correction_filted.y;
//		correction.z = correction_filted.z;

		// Gyro bias estimation
		bias->x += corr.x * Ki_acc;
		bias->y += corr.y * Ki_acc;
		bias->z += corr.z * Ki_acc;
		
		//Integral limiter
		if( bias->x > bias_max)
		{
			bias->x = bias_max;
		}		
		if( bias->x < -bias_max)
		{
			bias->x = -bias_max;
		}
	
		if( bias->y > bias_max)
		{
			bias->y = bias_max;
		}
		if( bias->y < -bias_max)
		{
			bias->y = -bias_max;
		}
	
		if( bias->z > bias_max)
		{
			bias->z = bias_max;
		}
		if( bias->z < -bias_max)
		{
			bias->z = -bias_max;
		}
		
		Quaternion dQ;
		Axis2Quat(&dQ,corr);
		Quat_Prod( q, *q, dQ );
	}
	
//	correction.x = 0.0f;
//	correction.y = 0.0f;
//	correction.z = 0.0f;
	
//	//陀螺积分
//	vector3 gyro;
//	gyro.x = (GimbalGyro_x - GyroOffset[0]/32.8f) * DEG2RAD + correction.x;
//	gyro.y = (GimbalGyro_y - GyroOffset[1]/32.8f) * DEG2RAD + correction.y;
//	gyro.z = (GimbalGyro_z - GyroOffset[2]/32.8f) * DEG2RAD + correction.z;
//	
//	vector3 delta_angle;
//	delta_angle.x = gyro.x * sample_time_gyro;
//	delta_angle.y = gyro.y * sample_time_gyro;
//	delta_angle.z = gyro.z * sample_time_gyro;
//	
//	integral( &Attitude_quat, delta_angle );
//	Quat2Euler( &euler_Est, *q );
}

float target_attitude_range_pitch(float max, vector3 angle_vehicle, vector3 encoder)
{
	//计算飞控姿态四元数
	Quaternion quat_fc;
	Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, angle_vehicle.z );
	//获取编码器角度并计算四元数
	Quaternion quat_encoder;
	Get_Encoder_Quat( &quat_encoder, encoder.x, max, encoder.z);
	//结合飞控姿态计算相机姿态
	Quaternion quat_measure;
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	
	vector3 euler;
	Quat2Euler( &euler, quat_measure );
	float pitch_max = euler.y;
	
	return pitch_max;
}

void target_attitude_range1(vector3* target_attitude)
{
	float pitch_max;
	pitch_max = target_attitude_range_pitch(target_attitude->y,Angle_Vehicle,Encoder);
	//pitch保护
	if(pitch_max > target_attitude->y)
	{
		target_attitude->y = pitch_max;
	}
}

void target_attitude_range2(vector3* target_attitude, vector3 angle_vehicle, vector3 encoder, Quaternion q)
{
	float pitch_min,roll_max,roll_min;
	float pitch_fc = angle_vehicle.y * RAD2DEG;
	float roll_fc = angle_vehicle.x * RAD2DEG;
	float yaw = atan2f( 2.0f*(q.qw*q.qz+q.qx*q.qy) , 1.0f-2.0f*(q.qy*q.qy+q.qz*q.qz) );//zxy的偏航
//	float yaw = atan2f( 2.0f*(q.qw*q.qz-q.qx*q.qy) , 1.0f-2.0f*(q.qx*q.qx+q.qz*q.qz) );//zyx的偏航
	float cos_yaw = cosf(yaw - angle_vehicle.z);
	float sin_yaw = sinf(yaw - angle_vehicle.z);
	
	//计算角度边界值
	pitch_min = -40.0f + cos_yaw * pitch_fc - sin_yaw * roll_fc;
	roll_max = 40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	roll_min = -40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	
	//目标姿态保护
	if(target_attitude->x * RAD2DEG < roll_min)
	{
		target_attitude->x = roll_min * DEG2RAD;
	}
	
	if(target_attitude->x * RAD2DEG > roll_max)
	{
		target_attitude->x = roll_max * DEG2RAD;
	}
	
	if(target_attitude->y * RAD2DEG < pitch_min)
	{
		target_attitude->y = pitch_min * DEG2RAD;
	}
}

void target_attitude_range3(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, Quaternion q)
{
	float roll_min,roll_max,pitch_min,pitch_max,yaw_min,yaw_max;
	float roll_fc = angle_vehicle.x * RAD2DEG;
	float pitch_fc = angle_vehicle.y * RAD2DEG;
	
	float cos_roll_fc = cosf(angle_vehicle.x);
	float cos_pitch_fc = cosf(angle_vehicle.y);
	
	float yaw = atan2f( 2.0f*(q.qw*q.qz+q.qx*q.qy) , 1.0f-2.0f*(q.qy*q.qy+q.qz*q.qz) );//zxy的偏航
//	float cos_yaw = cosf(yaw - angle_vehicle.z);
//	float sin_yaw = sinf(yaw - angle_vehicle.z);
	
	float sin_yaw = sinf(yaw_head*DEG2RAD);
	float cos_yaw = cosf(yaw_head*DEG2RAD);
	
	//计算角度边界值
	roll_min = -40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	roll_max = 40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	pitch_min = -40.0f + cos_yaw * pitch_fc - sin_yaw * roll_fc;
	pitch_max = 130.0f + cos_yaw * pitch_fc - sin_yaw * roll_fc;
	yaw_min = -170.0f ;
	yaw_max = 120.0f;
	
	//目标姿态保护
	if(target_attitude->x * RAD2DEG < roll_min)
	{
		target_attitude->x = roll_min * DEG2RAD;
	}
	
	if(target_attitude->x * RAD2DEG > roll_max)
	{
		target_attitude->x = roll_max * DEG2RAD;
	}
	
	if(target_attitude->y * RAD2DEG < pitch_min)
	{
		target_attitude->y = pitch_min * DEG2RAD;
	}
	
	if(target_attitude->y * RAD2DEG > pitch_max)
	{
		target_attitude->y = pitch_max * DEG2RAD;
	}
	
	if(yaw_head < yaw_min)
	{
		target_attitude->z = angle_vehicle.z + yaw_min * DEG2RAD;
	}
	
	if(yaw_head > yaw_max)
	{
		target_attitude->z = angle_vehicle.z + yaw_max * DEG2RAD;
	}
}

float roll_min = -40.0f;
float roll_max = 40.0f;
float pitch_min = -40.0f;
float pitch_max = 130.0f;
float yaw_min = -170.0f;
float yaw_max = 120.0f;
void target_attitude_range4(Quaternion* target_quat,vector3 target_attitude, vector3 angle_vehicle, vector3 encoder)
{
	bool protect = false;
	//电机角度保护
	if(encoder.x * RAD2DEG < roll_min)
	{
		encoder.x = roll_min * DEG2RAD;
		protect = true;
	}
	
	if(encoder.x * RAD2DEG > roll_max)
	{
		encoder.x = roll_max * DEG2RAD;
		protect = true;
	}
	
	if(encoder.y * RAD2DEG < pitch_min)
	{
		encoder.y = pitch_min * DEG2RAD;
		protect = true;
	}
	
	if(encoder.y * RAD2DEG > pitch_max)
	{
		encoder.y = pitch_max * DEG2RAD;
		protect = true;
	}
	
	if(encoder.z * RAD2DEG < yaw_min)
	{
		encoder.z = yaw_min * DEG2RAD;
		protect = true;
	}
	
	if(encoder.z * RAD2DEG > yaw_max)
	{
		encoder.z = yaw_max * DEG2RAD;
		protect = true;
	}
	
	if(protect == true)
	{
		//计算飞控姿态四元数
		Quaternion quat_fc;
		Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, angle_vehicle.z );
			
		//获取编码器角度并计算四元数
		Quaternion quat_encoder;
		Get_Encoder_Quat( &quat_encoder, encoder.x, encoder.y, encoder.z);
			
		//结合飞控姿态计算相机姿态
		Quat_Prod( target_quat, quat_fc, quat_encoder );
	}
	else
	{
		//按照正常目标角度计算姿态四元数
		Euler2Quat_G( target_quat, target_attitude.x, target_attitude.y, target_attitude.z);
	}		
}

void target_attitude_range5(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, vector3 encoder)
{
	float roll_min,roll_max,pitch_min,pitch_max,yaw_min,yaw_max;
	//计算飞控姿态四元数
	Quaternion quat_fc,quat_encoder,quat_measure;
	vector3 euler;
	Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, 0.0f );

/*****************计算RP最小角度****************/	
	//获取编码器角度并计算四元数
	Get_Encoder_Quat( &quat_encoder, -40.0f*DEG2RAD, -40.0f*DEG2RAD, yaw_head);
	//结合飞控姿态计算相机姿态
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	Quat2Euler_G( &euler, quat_measure );
	roll_min = euler.x;
	pitch_min = euler.y;
//	yaw_min = euler.z;
/*****************计算RP最小角度****************/	

/*****************计算RP最大角度****************/	
	//获取编码器角度并计算四元数
	Get_Encoder_Quat( &quat_encoder, 40.0f*DEG2RAD, 130.0f*DEG2RAD, yaw_head);
	//结合飞控姿态计算相机姿态
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	Quat2Euler_G( &euler, quat_measure );
	roll_max = euler.x;
	pitch_max = euler.y;
//	yaw_max = euler.z;
/*****************计算RP最大角度****************/

/*****************计算Y最小角度****************/
	//获取编码器角度并计算四元数
	Get_Encoder_Quat( &quat_encoder, encoder.x, encoder.y, -170.0f*DEG2RAD);
	//结合飞控姿态计算相机姿态
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	Quat2Euler_G( &euler, quat_measure );
	yaw_min = euler.z;

/*****************计算Y最小角度****************/

/*****************计算Y最大角度****************/
	//获取编码器角度并计算四元数
	Get_Encoder_Quat( &quat_encoder, encoder.x, encoder.y, 120.0f*DEG2RAD);
	//结合飞控姿态计算相机姿态
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	Quat2Euler_G( &euler, quat_measure );
	yaw_max = euler.z;

/*****************计算Y最大角度****************/

    //目标姿态保护
	if(target_attitude->x < roll_min)
	{
		target_attitude->x = roll_min;
	}
	
	if(target_attitude->x > roll_max)
	{
		target_attitude->x = roll_max;
	}
	
	if(target_attitude->y < pitch_min)
	{
		target_attitude->y = pitch_min;
	}
	
	if(target_attitude->y > pitch_max)
	{
		target_attitude->y = pitch_max;
	}
//	
//	if(yaw_head < -170.0f*DEG2RAD)
//	{
//		target_attitude->z = angle_vehicle.z + yaw_min;
//	}
//	
//	if(yaw_head > yaw_max)
//	{
//		target_attitude->z = angle_vehicle.z + yaw_max;
//	}
	
}

void target_attitude_range6(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, Quaternion q, vector3 encoder)
{
	float roll_min,roll_max,pitch_min,pitch_max,yaw_min,yaw_max;
	float roll_fc = angle_vehicle.x * RAD2DEG;
	float pitch_fc = angle_vehicle.y * RAD2DEG;
	
	float cos_roll_fc = cosf(angle_vehicle.x);
	float cos_pitch_fc = cosf(angle_vehicle.y);
	
//	float yaw = atan2f( 2.0f*(q.qw*q.qz+q.qx*q.qy) , 1.0f-2.0f*(q.qy*q.qy+q.qz*q.qz) );//zxy的偏航
//	float cos_yaw = cosf(yaw - angle_vehicle.z);
//	float sin_yaw = sinf(yaw - angle_vehicle.z);
	
//	float sin_yaw = sinf(yaw_head*DEG2RAD);
//	float cos_yaw = cosf(yaw_head*DEG2RAD);
	
	//计算yaw边界值
	yaw_min = -170.0f;
	yaw_max = 110.0f;
	
	//目标yaw保护
	if(yaw_head < yaw_min)
	{
		yaw_head = yaw_min;
		target_attitude->z = angle_vehicle.z + yaw_min * DEG2RAD;
	}
	
	if(yaw_head > yaw_max)
	{
		yaw_head = yaw_max;
		target_attitude->z = angle_vehicle.z + yaw_max * DEG2RAD;
	}
	
	//计算roll边界值
	float sin_yaw = sinf(yaw_head*DEG2RAD);
	float cos_yaw = cosf(yaw_head*DEG2RAD);
	roll_min = -40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	roll_max = 40.0f + sin_yaw * pitch_fc + cos_yaw * roll_fc;
	
	//目标roll保护
	if(target_attitude->x * RAD2DEG < roll_min)
	{
		target_attitude->x = roll_min * DEG2RAD;
	}
	
	if(target_attitude->x * RAD2DEG > roll_max)
	{
		target_attitude->x = roll_max * DEG2RAD;
	}
//	
	//计算pitch边界值
	Quaternion quat_fc,quat_encoder,quat_measure;
	vector3 euler;
	Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, 0.0f );

	//获取编码器角度并计算四元数
	Get_Encoder_Quat( &quat_encoder, encoder.x, -40.0f*DEG2RAD, encoder.z);
	//结合飞控姿态计算相机姿态
	Quat_Prod( &quat_measure, quat_fc, quat_encoder );
	Quat2Euler_G( &euler, quat_measure );
//	roll_min = euler.x;
	pitch_min = euler.y*RAD2DEG;
	pitch_max = 170.0f + pitch_min;
	
	//目标pitch保护
	if(target_attitude->y * RAD2DEG < pitch_min)
	{
		target_attitude->y = pitch_min * DEG2RAD;
	}
	
	if(target_attitude->y * RAD2DEG > pitch_max)
	{
		target_attitude->y = pitch_max * DEG2RAD;
	}
	
	debug_buf[3] = roll_min;
	debug_buf[4] = roll_max;
	debug_buf[5] = pitch_min;
	debug_buf[6] = pitch_max;
	debug_buf[7] = yaw_min;
	debug_buf[8] = yaw_max;
}

/*****************************************************
*原型: float Angle_Process(float angle)
*功能：将输入的角度转化到+-180度
*输入：
*输出：
******************************************************/
STM32F303_RAMFUNC float Angle_Process(float angle)
{
	if(angle > 180.0f)angle -= 360.0f;	
	else if(angle < -180.0f)angle += 360.0f;
	return angle;
}

u8 Point_count = 0;   //每次初始化要置0
float Angle_Vehicle_yaw_smooth;
void set_target_attitude(vector3* target_attitude, float roll, float pitch, float yaw)
{
	Quaternion q_temp;
	Euler2Quat( &q_temp, Angle_Vehicle.x, Angle_Vehicle.y, Angle_Vehicle.z );
	vector3 Angle_Vehicle_G;
	Quat2Euler_G( &Angle_Vehicle_G, q_temp );
	
	debug_buf[9] = Angle_Vehicle_G.x*RAD2DEG;
	debug_buf[10] = Angle_Vehicle_G.y*RAD2DEG;
	debug_buf[11] = Angle_Vehicle_G.z*RAD2DEG;
	
	if(Point_count == 0)
	{
		Angle_Vehicle_yaw_smooth = Angle_Vehicle_G.z*RAD2DEG;
		Point_count = 1;
	}
	else
	{
		float d_angle = 0.0f;
		d_angle = Angle_Process(Angle_Vehicle_G.z*RAD2DEG - Angle_Vehicle_yaw_smooth);
		float Kp_angle = 1.0f;
		Kp_angle = (PID_Parameter_Adjustment1(d_angle));
		Angle_Vehicle_yaw_smooth+= d_angle*powf(Kp_angle,1.5f)*dt;
	}
	
	vector3 Euler_Point;
	Euler_Point.x = roll;
	Euler_Point.y = pitch;
	Euler_Point.z = Angle_Vehicle_yaw_smooth + yaw;
	
	//角度规范化
	Euler_Point.x = Angle_Process(Euler_Point.x);
	Euler_Point.y = Angle_Process(Euler_Point.y);
	Euler_Point.z = Angle_Process(Euler_Point.z); 
	
//	float yaw_head = 0.0f;
//	vector3 target_attitude;
	target_attitude->x = Euler_Point.x * DEG2RAD;
	target_attitude->y = Euler_Point.y * DEG2RAD;
	target_attitude->z = Euler_Point.z * DEG2RAD;
//	target_attitude->z = Angle_Vehicle.z + yaw_head*DEG2RAD;
}
