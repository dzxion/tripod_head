#include "app.h"

u16 Gimbal_CaliData[320];

u16 Temp_PWM = 0;//加热电阻控制量

s8 Set_Temp = 75;

float TEMP_SPEED = 0.0f;
float TEMP_SPEED_A = 0.0f;

float TEMP_LastTime = 0.0f;

void Temp_pid(u8 SetTemp)//500us 执行一次
{	
	TEMP_SPEED_A = (MPU6500_Temp - TEMP_LastTime) / 0.0005f;
	TEMP_LastTime = MPU6500_Temp;
	
	TEMP_SPEED+=(TEMP_SPEED_A-TEMP_SPEED)*0.001f;
	
  PID_run_FloatspdVolt(&TempHeating_PID,SetTemp,MPU6500_Temp);	
	PID_run_FloatspdVolt(&TempSpeed_PID,TempHeating_PID.PID_Out,TEMP_SPEED);
	
	Temp_PWM = (u16)TempSpeed_PID.PID_Out;
	TIM_SetCompare1(TIM3,Temp_PWM);
}

void Gyro_Cali(void)
{
//	Temp_pid(Set_Temp);	//设定加热到75度
}


void Read_Cali_Data(void)
{
//	STMFLASH_Read(0X0801F800,Gimbal_CaliData,sizeof(Gimbal_CaliData)/2);	
}

