#ifndef __AHRS_H
#define __AHRS_H
#include "sys.h" 

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

extern uint8_t ReadMPU6500[14];

extern s16 Acc_x,Acc_y,Acc_z;
extern s16 Gyro_x, Gyro_y, Gyro_z;
extern float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
extern float pitch, roll, yaw;
extern float pitch_acc, roll_acc, yaw_acc;
extern float MPU6500_Temp;
extern int8_t Gyro_Temp;

void IMU_Update(void);
void MS_Attitude_Mahony(void);
void MS_Attitude_Acconly(void);
void MS_Attitude_GyroIntegral(void);
void init_MS_Attitude(void);

#endif
