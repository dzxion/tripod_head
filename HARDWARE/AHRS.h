#ifndef __AHRS_H
#define __AHRS_H
#include "sys.h" 

typedef struct
{
	float x;
	float y;
	float z;
}Vector3;

extern uint8_t ReadMPU6500[14];

extern s16 Acc_x,Acc_y,Acc_z;
extern s16 Gyro_x, Gyro_y, Gyro_z;
extern float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
extern float GyroOffset[3];
extern float pitch, roll, yaw;
extern float pitch_acc, roll_acc, yaw_acc;
extern float pitch_acc_filted, roll_acc_filted, yaw_acc_filted;
extern float MPU6500_Temp;
extern int8_t Gyro_Temp;
extern float q[4];
extern float gyro_bias[3];
extern float gyro[3];
extern float roll_fc,pitch_fc,yaw_fc;

void IMU_Update(void);
void MS_Attitude_Mahony(void);
void MS_Attitude_Acconly(void);
void MS_Attitude_GyroIntegral(void);
void init_MS_Attitude(void);
void cal_gyro_bias(void);
void MS_Attitude_Mahony_Bias(void);
void MS_Attitude_FC(void);

#endif
