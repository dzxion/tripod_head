#ifndef __AHRS_H
#define __AHRS_H
#include "sys.h" 
#include <stdbool.h>
#include "math_common.h"
#include "motor_control.h"

typedef struct
{
	float x;
	float y;
	float z;
}Vector3;

extern bool AHRS_OK;
extern vector3 gyro,acc,Encoder,Angle_Vehicle,acc_filted,euler_Est;
extern Quaternion qEstC,qEstC_fc,qEst;
extern vector3 biasC,biasC_fc,bias;
//extern float fs;
extern uint8_t ReadMPU6500[14];

extern s16 Acc_x,Acc_y,Acc_z;
extern s16 Gyro_x, Gyro_y, Gyro_z;
extern float GimbalGyro_x,GimbalGyro_y,GimbalGyro_z;
extern float GyroOffset[3];
extern float rollEstC, pitchEstC, yawEstC;
extern float rollEstC_fc,pitchEstC_fc,yawEstC_fc;
extern float rollEst,pitchEst,yawEst;
extern float pitch_acc, roll_acc, yaw_acc;
extern float pitch_acc_filted, roll_acc_filted, yaw_acc_filted;
extern float MPU6500_Temp;
extern int8_t Gyro_Temp;
extern float q[4];
extern float gyro_bias[3];
extern const float dt;
extern float bias_max;
//extern float gyro[3];

void Filter_LP_IIR_1(vector3* output, vector3 input);
void Get_Vehicle_Attitude(vector3* Attitude_Vehicle,Fd_InsData_t Attitude_Data);
void IMU_Update(vector3* gyro, vector3* acc);
void MS_Attitude_Acconly(vector3 acc, float* roll, float* pitch);
void MS_Attitude_GyroIntegral(Quaternion* q, vector3 gyro, vector3 bias, float dt);
void init_MS_Attitude(Quaternion* q);
void cal_gyro_bias(void);
void MS_Attitude_CF_Bias(Quaternion* q,vector3* bias,vector3 gyro,vector3 acc);
void MS_Attitude_CF_Bias_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder);
void Get_Encoder_Quat( Quaternion* q, float roll, float pitch, float yaw );
float target_attitude_range_pitch(float max, vector3 angle_vehicle, vector3 encoder);
void target_attitude_range1(vector3* target_attitude);
void target_attitude_range2(vector3* target_attitude, vector3 angle_vehicle, vector3 encoder, Quaternion q);
void target_attitude_range3(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, Quaternion q);
void target_attitude_range4(Quaternion* target_quat,vector3 target_attitude, vector3 angle_vehicle, vector3 encoder);
void target_attitude_range5(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, vector3 encoder);
void target_attitude_range6(vector3* target_attitude, vector3 angle_vehicle, float yaw_head, Quaternion q, vector3 encoder);
void set_target_attitude(vector3* target_attitude, float roll, float pitch, float yaw);

#endif
