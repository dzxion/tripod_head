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

extern PIDFloat_Obj Pitch_Angel_PID;
extern PIDFloat_Obj Pitch_Speed_PID;

extern PIDFloat_Obj Pitch_Merge_PID;
extern PIDFloat_Obj Roll_Merge_PID;
extern PIDFloat_Obj yaw_Merge_PID;

extern PIDFloat_Obj Roll_Angel_PID;
extern PIDFloat_Obj Roll_Speed_PID;

extern PIDFloat_Obj Yaw_Angel_PID;
extern PIDFloat_Obj Yaw_Speed_PID;

extern PIDFloat_Obj TempHeating_PID;
extern PIDFloat_Obj TempSpeed_PID;

void Gimbal_Init(void);
void Para_Init(void);

STM32F303_RAMFUNC float PID_run_FloatspdVolt(PIDFloat_Obj* handle,float GivenAngle,float FeedbackAngle);
	
#endif
