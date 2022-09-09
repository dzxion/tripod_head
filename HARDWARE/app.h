#ifndef __app_H
#define __app_H

#include "Debug.h"
#include "MPU6500.h"
#include "AHRS.h"
#include "Motor_Tim.h"
#include "FOC.h"
#include "can.h"
#include "port.h"
#include "stmflash.h"
#include "Gpio_Config.h"
#include "Calibration.h"
#include "Motor_Control.h"
#include "led.h"
#include "string.h"
#include "usart.h" 
#include "arm_math.h"
#include "delay.h"
#include "timer.h"
#include "stm32f30x_dma.h"
#include "stm32f30x.h"
#include "stm32f30x_rtc.h"
#include <stdlib.h>
#include "stdio.h"
#include "superx_ii_api.h"
#include "cpu.h"
#include <stdbool.h>

#define VERSION(ver, sub,reg, biuld) (((uint32_t)ver << 24)|((uint32_t)sub << 16)|((uint32_t)reg << 8)|(biuld & 0xFF))

#define HARDWARE_VERSION VERSION(1, 0, 0, 1)
#define SOFTWARE_VERSION VERSION(1, 0, 0, 14)

typedef struct
{  
	volatile float Kp;
  volatile float Ki;

	volatile float last_error;
	volatile float I_Out;
	
	volatile float P_Min;
  volatile float P_Max;
	
	volatile float I_Min;
  volatile float I_Max;
	
  volatile float outMin;
  volatile float outMax;
 
	float PID_Out;
	
	float PID_P;
	float PID_RY;
	float Out_Filter;
}PIDFloat_Obj;

typedef struct
{
	float Angle_speed_P;
	float Angle_speed_R;
	float Angle_speed_Y;
}Angle_speed_t;

typedef struct
{
	float Angle_P;
	float Angle_R;
	float Angle_Y;
}Encoder_t;

extern PIDFloat_Obj Pitch_Angel_PID;
extern PIDFloat_Obj Pitch_Speed_PID;
extern PIDFloat_Obj Pitch_Encoder_PID;

extern PIDFloat_Obj Pitch_Merge_PID;
extern PIDFloat_Obj Roll_Merge_PID;
extern PIDFloat_Obj yaw_Merge_PID;

extern PIDFloat_Obj Roll_Angel_PID;
extern PIDFloat_Obj Roll_Speed_PID;
extern PIDFloat_Obj Roll_Encoder_PID;

extern PIDFloat_Obj Yaw_Angel_PID;
extern PIDFloat_Obj Yaw_Speed_PID;
extern PIDFloat_Obj Yaw_Encoder_PID;

extern PIDFloat_Obj TempHeating_PID;
extern PIDFloat_Obj TempSpeed_PID;

extern vector3 target_angular_rate;

extern float pitch_encoder, roll_encoder, yaw_encoder;
extern float pitch_by_encoder, roll_by_encoder, yaw_by_encoder;
extern float target_angular_rate_body[3];
extern float debug_yaw;
extern float debug_yaw_angle;
extern float debug_buf[16];
extern bool calib_gyroscope;
extern u8 ahrs_count;
extern vector3 current_angular_rate;
extern Quaternion target_quat;

void Gimbal_Init(void);
void Para_Init(void);

void ctrl_angular_velocity(vector3 target_angular_rate,vector3 current_angular_rate);
void ctrl_Attitude(void);
void ctrl_Attitude_Q(vector3* u, Quaternion target_quat, Quaternion current_quat);
void ctrl_Attitude_decoup(Quaternion target_quat, Quaternion current_quat, vector3 gyro, vector3 encoder);
void ctrl_encoder(void);
STM32F303_RAMFUNC float PID_run_FloatspdVolt(PIDFloat_Obj* handle,float GivenAngle,float FeedbackAngle);
STM32F303_RAMFUNC float PID_Parameter_Adjustment1(float follow);

	
#endif
