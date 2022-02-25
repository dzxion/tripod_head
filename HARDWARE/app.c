#include "app.h"

PIDFloat_Obj Pitch_Angel_PID;
PIDFloat_Obj Pitch_Speed_PID;

PIDFloat_Obj Roll_Angel_PID;
PIDFloat_Obj Roll_Speed_PID;

PIDFloat_Obj Yaw_Angel_PID;
PIDFloat_Obj Yaw_Speed_PID;

PIDFloat_Obj TempSpeed_PID;
PIDFloat_Obj TempHeating_PID;

API_Config_t Gimbal_Config;

void ComProtocol(void)
{
	 uint32_t cpu_id[3]; 
	 mcu_get_uid(cpu_id);
   memset(&Gimbal_Config, 0, sizeof(API_Config_t));
   memcpy(&Gimbal_Config.ModuleID, (uint8_t*)cpu_id, 12);
	 Gimbal_Config.ModuleType = MODULE_TYPE_GIMBAL;
   Gimbal_Config.HardwareVersion = HARDWARE_VERSION;
   Gimbal_Config.SoftwareVersion = SOFTWARE_VERSION;
   Gimbal_Config.DataSize = 28;
   Gimbal_Config.ModuleTimeout = 2000;
   api_init(&Gimbal_Config);
}

#define DutyMax  500.0f; //角速度限制

void Gimbal_Init(void)
{
	Para_Init();
	Gpio_config();	
	LED_Init();
	USART3_init(1000000);
	if(Selection_axis == 0)
	{
		CanHardwareInit(CAN_BAUDRATE_1M);
		delay_ms(1);
		ComProtocol();
		SPI3_Init();
		Heating_Drive(500-1,72-1);//温度PWM
		TIM4_Int_Init(500-1,72-1); //周期2KHZ
		USART1_init(230400);//调试串口
		Pitch_ON;
	}
	else
	{
		TIM15_PWM_Init(0xffff,72-1);   //磁编码输入捕获
		SVPWM8_Init();
		Roll_Yaw_ON;
	}
	SVPWM1_Init();
	TIM2_PWM_Init(0xffff,72-1);  //磁编码输入捕获
}

void PID_Para_Init(void)
{
/*************************************************************/	
	Pitch_Angel_PID.Kp = 0.0f;//2.0
	Pitch_Angel_PID.Ki = 0.0f; //0.25 
	
	Pitch_Angel_PID.P_Min = -DutyMax;
	Pitch_Angel_PID.P_Max =  DutyMax;	
	
	Pitch_Angel_PID.I_Min = -DutyMax; 
	Pitch_Angel_PID.I_Max =  DutyMax;
	
	Pitch_Angel_PID.outMin = -DutyMax;
	Pitch_Angel_PID.outMax =  DutyMax;	
	
	Pitch_Angel_PID.I_Out = 0.0f; 
	Pitch_Angel_PID.PID_Out = 0.0f;
/*************************************************************/		
	Pitch_Speed_PID.Kp = 0.0075f; 
	Pitch_Speed_PID.Ki = 0.00075f; 
	
	Pitch_Speed_PID.P_Min = -1.0f;
	Pitch_Speed_PID.P_Max =  1.0f;	
	
	Pitch_Speed_PID.I_Min = -1.0f; 
	Pitch_Speed_PID.I_Max =  1.0f;
	
	Pitch_Speed_PID.outMin = -1.0f;
	Pitch_Speed_PID.outMax =  1.0f;	
	
	Pitch_Speed_PID.I_Out = 0.0f; 
	Pitch_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	


/*************************************************************/	
	Roll_Angel_PID.Kp = 0.0f;//2.0
	Roll_Angel_PID.Ki = 0.0f;  //0.25
	
	Roll_Angel_PID.P_Min = -DutyMax;
	Roll_Angel_PID.P_Max =  DutyMax;	
	
	Roll_Angel_PID.I_Min = -DutyMax; 
	Roll_Angel_PID.I_Max =  DutyMax;
	
	Roll_Angel_PID.outMin = -DutyMax;
	Roll_Angel_PID.outMax =  DutyMax;	
	
	Roll_Angel_PID.I_Out = 0.0f; 
	Roll_Angel_PID.PID_Out = 0.0f;
/*************************************************************/		
	Roll_Speed_PID.Kp = 0.01f; //0.01
	Roll_Speed_PID.Ki = 0.001f; 
	
	Roll_Speed_PID.P_Min = -1.0f;
	Roll_Speed_PID.P_Max =  1.0f;	
	
	Roll_Speed_PID.I_Min = -1.0f; 
	Roll_Speed_PID.I_Max =  1.0f;
	
	Roll_Speed_PID.outMin = -1.0f;
	Roll_Speed_PID.outMax =  1.0f;	
	
	Roll_Speed_PID.I_Out = 0.0f; 
	Roll_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	


/*************************************************************/	
	Yaw_Angel_PID.Kp = 0.0f;//1.0
	Yaw_Angel_PID.Ki = 0.0f; //0.125 
	
	Yaw_Angel_PID.P_Min = -DutyMax;
	Yaw_Angel_PID.P_Max =  DutyMax;	
	
	Yaw_Angel_PID.I_Min = -DutyMax; 
	Yaw_Angel_PID.I_Max =  DutyMax;
	
	Yaw_Angel_PID.outMin = -DutyMax;
	Yaw_Angel_PID.outMax =  DutyMax;	
	
	Yaw_Angel_PID.I_Out = 0.0f; 
	Yaw_Angel_PID.PID_Out = 0.0f;
/*************************************************************/		
	Yaw_Speed_PID.Kp = 0.01f; //0.01
	Yaw_Speed_PID.Ki = 0.001f; 
	
	Yaw_Speed_PID.P_Min = -1.0f;
	Yaw_Speed_PID.P_Max =  1.0f;	
	
	Yaw_Speed_PID.I_Min = -1.0f; 
	Yaw_Speed_PID.I_Max =  1.0f;
	
	Yaw_Speed_PID.outMin = -1.0f;
	Yaw_Speed_PID.outMax =  1.0f;	
	
	Yaw_Speed_PID.I_Out = 0.0f; 
	Yaw_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	


/*************************************************************/	
	TempHeating_PID.Kp = 0.18f;
	TempHeating_PID.Ki = 0.0f;  
	
	TempHeating_PID.P_Min = -0.25f;
	TempHeating_PID.P_Max =  0.25f;	
	
	TempHeating_PID.I_Min = -0.25f; 
	TempHeating_PID.I_Max =  0.25f;
	
	TempHeating_PID.outMin =  -0.25f;
	TempHeating_PID.outMax =  0.25f;	
	
	TempHeating_PID.I_Out = 0.0f; 
	TempHeating_PID.PID_Out = 0.0f;
/*************************************************************/	

/*************************************************************/	
	TempSpeed_PID.Kp = 10.0f;
	TempSpeed_PID.Ki = 0.025f;  
	
	TempSpeed_PID.P_Min =  0.0f;
	TempSpeed_PID.P_Max =  500.0f;	
	
	TempSpeed_PID.I_Min =  0.0f; 
	TempSpeed_PID.I_Max =  500.0f;
	
	TempSpeed_PID.outMin = 0.0f;
	TempSpeed_PID.outMax =  500.0f;	
	
	TempSpeed_PID.I_Out = 0.0f; 
	TempSpeed_PID.PID_Out = 0.0f;
/*************************************************************/		
}

void Para_Init(void)
{
	PID_Para_Init();
	
  Get_Encoder.Angle_P = 0.0f;
	Get_Encoder.Angle_R = 0.0f;
	Get_Encoder.Angle_Y = 0.0f;	
	
	ThetaOffset.P = 96;
	ThetaOffset.R = 160;
	ThetaOffset.Y = 212;
}

STM32F303_RAMFUNC float PID_run_FloatspdVolt(PIDFloat_Obj* handle,float GivenAngle,float FeedbackAngle)
{
	float Error_value;

	float P_Out;
	
	Error_value = GivenAngle - FeedbackAngle;
	
	P_Out = Error_value * handle->Kp;
	handle->I_Out += Error_value * handle->Ki;

	if(P_Out < handle->P_Min)P_Out = handle->P_Min; 
	else if(P_Out > handle->P_Max)P_Out = handle->P_Max;
	
	if(handle->I_Out < handle->I_Min)handle->I_Out = handle->I_Min;
	else if(handle->I_Out > handle->I_Max)handle->I_Out = handle->I_Max;
  	
	handle->PID_Out = P_Out + handle->I_Out;
	
	if(handle->PID_Out > handle->outMax)
	{
    handle->PID_Out = handle->outMax;
	}
  else if(handle->PID_Out < handle->outMin)
	{
    handle->PID_Out = handle->outMin;
	}
	return handle->PID_Out;
}

STM32F303_RAMFUNC float PID_run(PIDFloat_Obj* handle,float Error_value)
{
	float P_Out;
	
	P_Out = Error_value * handle->Kp;
	handle->I_Out += Error_value * handle->Ki;

	if(P_Out < handle->P_Min)P_Out = handle->P_Min; 
	else if(P_Out > handle->P_Max)P_Out = handle->P_Max;
	
	if(handle->I_Out < handle->I_Min)handle->I_Out = handle->I_Min;
	else if(handle->I_Out > handle->I_Max)handle->I_Out = handle->I_Max;
  	
	handle->PID_Out = P_Out + handle->I_Out;
	
	if(handle->PID_Out > handle->outMax)
	{
    handle->PID_Out = handle->outMax;
	}
    else if(handle->PID_Out < handle->outMin)
	{
    handle->PID_Out = handle->outMin;
	}
	return handle->PID_Out;
}

void ctrl_angular_velocity(float target_angular_velocity_x,float target_angular_velocity_y,float target_angular_velocity_z,
                           float current_angular_velocity_x,float current_angular_velocity_y,float current_angular_velocity_z)
{
	//计算角速度误差
	float error_angular_velocity_body_x = target_angular_velocity_x - current_angular_velocity_x;
	float error_angular_velocity_body_y = target_angular_velocity_y - current_angular_velocity_y;
	float error_angular_velocity_body_z = target_angular_velocity_z - current_angular_velocity_z;
	
	//机体误差映射到三个电机上
	float sin_roll, cos_roll;
	float sin_pitch, cos_pitch;
	sin_roll = sin(roll_encoder);cos_roll = cos(roll_encoder);
	sin_pitch = sin(pitch_encoder);cos_pitch = cos(pitch_encoder);
	float inv_cos_roll = 1.0f / cos_roll;
	
	float error_motor_speed_roll = cos_pitch * error_angular_velocity_body_x + sin_pitch * error_angular_velocity_body_z;
	float error_motor_speed_pitch = error_angular_velocity_body_y + (sin_roll * sin_pitch * error_angular_velocity_body_x - sin_roll * cos_pitch * error_angular_velocity_body_z) * inv_cos_roll;
	float error_motor_speed_yaw = (- sin_pitch * error_angular_velocity_body_x + cos_pitch * error_angular_velocity_body_z) * inv_cos_roll;
	
	//PID控制
	PID_run(&Roll_Speed_PID, error_motor_speed_roll);
	PID_run(&Pitch_Speed_PID, error_motor_speed_pitch);
	PID_run(&Yaw_Speed_PID, error_motor_speed_yaw);
}
