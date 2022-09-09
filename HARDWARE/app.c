#include "app.h"
#include <float.h>
#include <stdbool.h>
#include "math_common.h"

PIDFloat_Obj Pitch_Angel_PID;
PIDFloat_Obj Pitch_Speed_PID;
PIDFloat_Obj Pitch_Encoder_PID;

PIDFloat_Obj Roll_Angel_PID;
PIDFloat_Obj Roll_Speed_PID;
PIDFloat_Obj Roll_Encoder_PID;

PIDFloat_Obj Yaw_Angel_PID;
PIDFloat_Obj Yaw_Angel_e_PID;
PIDFloat_Obj Yaw_Speed_PID;
PIDFloat_Obj Yaw_Encoder_PID;

PIDFloat_Obj TempSpeed_PID;
PIDFloat_Obj TempHeating_PID;

API_Config_t Gimbal_Config;

Angle_speed_t motor_speed;
Encoder_t encoder;

vector3 target_angular_rate = {0.0f,0.0f,0.0f};//Ŀ�����ݽ��ٶȣ�deg/s��

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

#define DutyMax  500.0f; //���ٶ�����

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
		Heating_Drive(500-1,72-1);//�¶�PWM
		TIM4_Int_Init(500-1,72-1); //����2KHZ
		USART1_init(230400);//���Դ���
		Pitch_ON;
	}
	else
	{
		TIM15_PWM_Init(0xffff,72-1);   //�ű������벶��
		SVPWM8_Init();
		Roll_Yaw_ON;
	}
	SVPWM1_Init();
	TIM2_PWM_Init(0xffff,72-1);  //�ű������벶��
}

void PID_Para_Init(void)
{
/*************************************************************/	
	Pitch_Angel_PID.Kp = 10.0f;//2.0
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
	Pitch_Speed_PID.Kp = 0.0045f; 
	Pitch_Speed_PID.Ki = 0.00045f; 
	
	Pitch_Speed_PID.P_Min = -1.0f;
	Pitch_Speed_PID.P_Max =  1.0f;	
	
	Pitch_Speed_PID.I_Min = -1.0f; 
	Pitch_Speed_PID.I_Max =  1.0f;
	
	Pitch_Speed_PID.outMin = -1.0f;
	Pitch_Speed_PID.outMax =  1.0f;	
	
	Pitch_Speed_PID.I_Out = 0.0f; 
	Pitch_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	
//	Pitch_Encoder_PID.Kp = 0.01f; 
//	Pitch_Encoder_PID.Ki = 0.0f; 
//	
//	Pitch_Encoder_PID.P_Min = -1.0f;
//	Pitch_Encoder_PID.P_Max =  1.0f;	
//	
//	Pitch_Encoder_PID.I_Min = -1.0f; 
//	Pitch_Encoder_PID.I_Max =  1.0f;
//	
//	Pitch_Encoder_PID.outMin = -1.0f;
//	Pitch_Encoder_PID.outMax =  1.0f;	
//	
//	Pitch_Encoder_PID.I_Out = 0.0f; 
//	Pitch_Encoder_PID.PID_Out = 0.0f;

/*************************************************************/	
	Roll_Angel_PID.Kp = 10.0f;//2.0
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
	Roll_Speed_PID.Kp = 0.0045f; //0.01
	Roll_Speed_PID.Ki = 0.00045f; 
	
	Roll_Speed_PID.P_Min = -1.0f;
	Roll_Speed_PID.P_Max =  1.0f;	
	
	Roll_Speed_PID.I_Min = -1.0f; 
	Roll_Speed_PID.I_Max =  1.0f;
	
	Roll_Speed_PID.outMin = -1.0f;
	Roll_Speed_PID.outMax =  1.0f;	
	
	Roll_Speed_PID.I_Out = 0.0f; 
	Roll_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	
//	Roll_Encoder_PID.Kp = 0.01f; //0.01
//	Roll_Encoder_PID.Ki = 0.0f; 
//	
//	Roll_Encoder_PID.P_Min = -1.0f;
//	Roll_Encoder_PID.P_Max =  1.0f;	
//	
//	Roll_Encoder_PID.I_Min = -1.0f; 
//	Roll_Encoder_PID.I_Max =  1.0f;
//	
//	Roll_Encoder_PID.outMin = -1.0f;
//	Roll_Encoder_PID.outMax =  1.0f;	
//	
//	Roll_Encoder_PID.I_Out = 0.0f; 
//	Roll_Encoder_PID.PID_Out = 0.0f;

/*************************************************************/	
	Yaw_Angel_PID.Kp = 10.0f;//1.0
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
//	Yaw_Angel_e_PID.Kp = 5.0f;//1.0
//	Yaw_Angel_e_PID.Ki = 0.0f; //0.125 
//	
//	Yaw_Angel_e_PID.P_Min = -DutyMax;
//	Yaw_Angel_e_PID.P_Max =  DutyMax;	
//	
//	Yaw_Angel_e_PID.I_Min = -DutyMax; 
//	Yaw_Angel_e_PID.I_Max =  DutyMax;
//	
//	Yaw_Angel_e_PID.outMin = -DutyMax;
//	Yaw_Angel_e_PID.outMax =  DutyMax;	
//	
//	Yaw_Angel_e_PID.I_Out = 0.0f; 
//	Yaw_Angel_e_PID.PID_Out = 0.0f;
	
/*************************************************************/		
	Yaw_Speed_PID.Kp = 0.002f; //0.01
	Yaw_Speed_PID.Ki = 0.0002f; 
	
	Yaw_Speed_PID.P_Min = -1.0f;
	Yaw_Speed_PID.P_Max =  1.0f;	
	
	Yaw_Speed_PID.I_Min = -1.0f; 
	Yaw_Speed_PID.I_Max =  1.0f;
	
	Yaw_Speed_PID.outMin = -1.0f;
	Yaw_Speed_PID.outMax =  1.0f;	
	
	Yaw_Speed_PID.I_Out = 0.0f; 
	Yaw_Speed_PID.PID_Out = 0.0f;
/*************************************************************/	
//	Yaw_Encoder_PID.Kp = 0.01f; //0.01
//	Yaw_Encoder_PID.Ki = 0.0f; 
//	
//	Yaw_Encoder_PID.P_Min = -1.0f;
//	Yaw_Encoder_PID.P_Max =  1.0f;	
//	
//	Yaw_Encoder_PID.I_Min = -1.0f; 
//	Yaw_Encoder_PID.I_Max =  1.0f;
//	
//	Yaw_Encoder_PID.outMin = -1.0f;
//	Yaw_Encoder_PID.outMax =  1.0f;	
//	
//	Yaw_Encoder_PID.I_Out = 0.0f; 
//	Yaw_Encoder_PID.PID_Out = 0.0f;

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

float limit_up = 10.0f;
float limit_down = 0.01f;
float Adj_k2 = 0.05f;//0.02f;
float Adj_kk = 0.1f;//(1/�Ƕ�)
float PID_Parameter_Adjustment1(float  Encoder_follow)
{
        float demp,demp_in;
        demp_in = fabs(Encoder_follow);
        demp = demp_in*Adj_kk;

        if(demp > limit_up)
        {
                demp = limit_up;        
        }
        if(demp < limit_down)
        {
                demp = limit_down;        
        }
        return demp;
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

void Body2Motor(vector3* vec_out, vector3 vec_in, vector3 encoder)
{
	float sin_roll = 0.0f, cos_roll = 0.0f, tan_roll = 0.0f;
	float sin_pitch = 0.0f, cos_pitch = 0.0f;
 
	sin_roll = sinf(encoder.x);
	cos_roll = cosf(encoder.x);
	sin_pitch = sinf(encoder.y);
	cos_pitch = cosf(encoder.y);
	tan_roll = sin_roll / cos_roll;
//	tan_roll = tanf(encoder.x);
	
	float inv_cos_roll = 1.0f / cos_roll;

	vec_out->x =           cos_pitch*vec_in.x                    + sin_pitch*vec_in.z;
	vec_out->y =  tan_roll*sin_pitch*vec_in.x  + vec_in.y - tan_roll*cos_pitch*vec_in.z;
	vec_out->z = -sin_pitch*inv_cos_roll*vec_in.x             + cos_pitch*inv_cos_roll*vec_in.z; 
	
//	vec_out->x =           cos_pitch*vec_in.x                      + sin_pitch*vec_in.z;
//	vec_out->y =  sin_roll*sin_pitch/cos_roll*vec_in.x  + vec_in.y - sin_roll*cos_pitch/cos_roll*vec_in.z;
//	vec_out->z = -sin_pitch/cos_roll*vec_in.x             + cos_pitch/cos_roll*vec_in.z; 
	
//	vec_out->x = cos_pitch * vec_in.x + sin_pitch * vec_in.z;
//	vec_out->y = vec_in.y + (sin_roll * sin_pitch * vec_in.x - sin_roll * cos_pitch * vec_in.z) * inv_cos_roll;
//	vec_out->z = (- sin_pitch * vec_in.x + cos_pitch * vec_in.z) * inv_cos_roll;
} 

/****************************************************************************************
*ԭ  �ͣ�void ctrl_angular_velocity(vector3 target_angular_rate,vector3 current_angular_rate)
*��  �ܣ����ٶȿ���
*��  �룺target_angular_rate :Ŀ����ٶ�
*��  �룺current_angular_rate :��ǰ��Ԫ��
*��  ������
*****************************************************************************************/
void ctrl_angular_velocity(vector3 target_angular_rate,vector3 current_angular_rate)
{
	//������ٶ����
//	float error_angular_velocity_body_x = target_angular_velocity_x - current_angular_velocity_x;
//	float error_angular_velocity_body_y = target_angular_velocity_y - current_angular_velocity_y;
//	float error_angular_velocity_body_z = target_angular_velocity_z - current_angular_velocity_z;
	
	vector3 error_angular_velocity_body = {0.0f,0.0f,0.0f};
	error_angular_velocity_body.x = target_angular_rate.x - current_angular_rate.x;
	error_angular_velocity_body.y = target_angular_rate.y - current_angular_rate.y;
	error_angular_velocity_body.z = target_angular_rate.z - current_angular_rate.z;
	
	//�������ӳ�䵽���������
//	float sin_roll, cos_roll;
//	float sin_pitch, cos_pitch;
//	sin_roll = sinf(roll_encoder * DEG2RAD);
//	cos_roll = cosf(roll_encoder * DEG2RAD);
//	sin_pitch = sinf(pitch_encoder * DEG2RAD);
//	cos_pitch = cosf(pitch_encoder * DEG2RAD);
//	float inv_cos_roll = 1.0f / cos_roll;
//	
//	float error_motor_speed_roll = cos_pitch * error_angular_velocity_body_x + sin_pitch * error_angular_velocity_body_z;
//	float error_motor_speed_pitch = error_angular_velocity_body_y + (sin_roll * sin_pitch * error_angular_velocity_body_x - sin_roll * cos_pitch * error_angular_velocity_body_z) * inv_cos_roll;
//	float error_motor_speed_yaw = (- sin_pitch * error_angular_velocity_body_x + cos_pitch * error_angular_velocity_body_z) * inv_cos_roll;
	
	vector3 error_motor_speed = {0.0f,0.0f,0.0f};
 	Body2Motor(&error_motor_speed, error_angular_velocity_body, Encoder);
	
	//PID����
	PID_run(&Roll_Speed_PID, error_motor_speed.x);
	PID_run(&Pitch_Speed_PID, error_motor_speed.y);
	PID_run(&Yaw_Speed_PID, error_motor_speed.z);
}

float target_Roll = 0.0f, target_Pitch = 0.0f, target_Yaw = 0.0f;
float target_angular_rate_body[3];
float Ps = 1.0f;
bool Yaw_Control_Enabled = false;

void ctrl_Attitude(void)
{
	float current_quat_PR[4];
	float q_norm;
	
	//��Ԫ���ֽ�(����1)
////	float AirframeQuat[4] = {q[0],q[1],q[2],q[3]};
//	float Yaw = atan2f( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );
//	float half_sinYaw, half_cosYaw;
//	half_sinYaw = sinf(0.5f*Yaw); half_cosYaw = cosf(0.5f*Yaw);
////	float YawQuat[4] = {half_cosYaw,0,0,-half_sinYaw};
//	
//	current_quat_PR[0] = q[0]*half_cosYaw + q[3]*half_sinYaw;
//	current_quat_PR[1] = q[1]*half_cosYaw + q[2]*half_sinYaw;
//	current_quat_PR[2] = q[2]*half_cosYaw - q[1]*half_sinYaw;
//	current_quat_PR[3] = q[3]*half_cosYaw - q[0]*half_sinYaw;
//	
//	q_norm = sqrtf(current_quat_PR[0]*current_quat_PR[0] + current_quat_PR[1]*current_quat_PR[1] + current_quat_PR[2]*current_quat_PR[2] + current_quat_PR[3]*current_quat_PR[3]);
//	current_quat_PR[0] = current_quat_PR[0] / q_norm;
//	current_quat_PR[1] = current_quat_PR[1] / q_norm;
//	current_quat_PR[2] = current_quat_PR[2] / q_norm;
//	current_quat_PR[3] = current_quat_PR[3] / q_norm;
	
	//��Ԫ���ֽ�(����2)��bug
	float tqx;
	float tqy;
	float tqw;
	float qw2 = q[0] * q[0];
	float qx2 = q[1] * q[1];
	float qy2 = q[2] * q[2];
	float qz2 = q[3] * q[3];
	float qwx = q[0] * q[1];
	float qwy = q[0] * q[2];
	float qwz = q[0] * q[3];
	float qxy = q[1] * q[2];
	float qxz = q[1] * q[3];
	float qyz = q[2] * q[3];
	
	float qw2Pqz2 = ( qw2 + qz2 );
	if (!(fabsf(qw2Pqz2) < FLT_EPSILON))
	{
		tqw = sqrtf( qw2Pqz2 );
		float inv_tqw = 1.0f / tqw;
		tqx = ( qwx + qyz ) * inv_tqw;
		tqy = ( qwy - qxz ) * inv_tqw;	
	}
	else			
	{
		//glimbal lock effect
		tqw = 0.0f;
		tqx = q[1];	tqy = q[2];
	}
	q_norm = sqrtf(tqw*tqw + tqx*tqx + tqy*tqy);
	current_quat_PR[0] = tqw / q_norm;
	current_quat_PR[1] = tqx / q_norm;
	current_quat_PR[2] = tqy / q_norm;
	current_quat_PR[3] = 0.0f;
	
	//���㵱ǰ��Ԫ����ת����
//	float PR_rotation_now[3];
//	float theta_now = 2.0f* acosf( current_quat_PR[0] );
//	if(theta_now > PI)
//		theta_now -= 2.0f*PI;
//	float sin_half_theta_now = sqrtf( 1.0f - current_quat_PR[0]*current_quat_PR[0] );
//	float scale_now;
//	if (fabsf(sin_half_theta_now) < FLT_EPSILON)
//		scale_now = 0.5f;
//	else
//		scale_now = theta_now / sin_half_theta_now;
//	PR_rotation_now[0] = current_quat_PR[1] * scale_now * RAD2DEG;
//	PR_rotation_now[1] = current_quat_PR[2] * scale_now * RAD2DEG;
//	PR_rotation_now[2] = current_quat_PR[3] * scale_now * RAD2DEG;
	
	//��Ԫ���ֽ�(����3)
//	float half_sinR_now, half_cosR_now;
//	half_sinR_now = sinf(0.5f*roll); half_cosR_now = cosf(0.5f*roll);
//	float half_sinP_now, half_cosP_now;
//	half_sinP_now = sinf(0.5f*pitch); half_cosP_now = cosf(0.5f*pitch);
//	
//	current_quat_PR[0] = half_cosR_now*half_cosP_now;
//	current_quat_PR[1] = half_cosP_now*half_sinR_now;
//	current_quat_PR[2] = half_cosR_now*half_sinP_now;
//	current_quat_PR[3] = -half_sinR_now*half_sinP_now;
//	
//	q_norm = sqrtf(current_quat_PR[0]*current_quat_PR[0] + current_quat_PR[1]*current_quat_PR[1] + current_quat_PR[2]*current_quat_PR[2] + current_quat_PR[3]*current_quat_PR[3]);
//	current_quat_PR[0] = current_quat_PR[0] / q_norm;
//	current_quat_PR[1] = current_quat_PR[1] / q_norm;
//	current_quat_PR[2] = current_quat_PR[2] / q_norm;
//	current_quat_PR[3] = current_quat_PR[3] / q_norm;
	
	//������ת����(body->world)
	current_quat_PR[1] = -current_quat_PR[1];current_quat_PR[2] = -current_quat_PR[2];current_quat_PR[3] = -current_quat_PR[3];
	float Rotation_Matrix[3][3];
	qw2 = current_quat_PR[0] * current_quat_PR[0];
	qx2 = current_quat_PR[1] * current_quat_PR[1];
	qy2 = current_quat_PR[2] * current_quat_PR[2];
	qz2 = current_quat_PR[3] * current_quat_PR[3];
	qwx = current_quat_PR[0] * current_quat_PR[1];
	qwy = current_quat_PR[0] * current_quat_PR[2];
	qwz = current_quat_PR[0] * current_quat_PR[3];
	qxy = current_quat_PR[1] * current_quat_PR[2];
	qxz = current_quat_PR[1] * current_quat_PR[3];
	qyz = current_quat_PR[2] * current_quat_PR[3];
	Rotation_Matrix[0][0]=qw2+qx2-qy2-qz2;	Rotation_Matrix[0][1]=2.0f*(qxy-qwz);	Rotation_Matrix[0][2]=2.0f*(qwy+qxz);
	Rotation_Matrix[1][0]=2.0f*(qwz+qxy);	Rotation_Matrix[1][1]=qw2-qx2+qy2-qz2;	Rotation_Matrix[1][2]=2.0f*(qyz-qwx);
	Rotation_Matrix[2][0]=2.0f*(qxz-qwy);	Rotation_Matrix[2][1]=2.0f*(qwx+qyz);	Rotation_Matrix[2][2]=qw2-qx2-qy2+qz2;
	current_quat_PR[1] = -current_quat_PR[1];current_quat_PR[2] = -current_quat_PR[2];current_quat_PR[3] = -current_quat_PR[3];
	
	//ʹ��Ŀ��Ƕȹ���Ŀ����Ԫ��
	float target_quat_PR[4];
	float half_sinR, half_cosR;
	half_sinR = sinf(0.5f*target_Roll); half_cosR = cosf(0.5f*target_Roll);
	float half_sinP, half_cosP;
	half_sinP = sinf(0.5f*target_Pitch); half_cosP = cosf(0.5f*target_Pitch);
	
	target_quat_PR[0] = half_cosR*half_cosP;
	target_quat_PR[1] = half_cosP*half_sinR;
	target_quat_PR[2] = half_cosR*half_sinP;
	target_quat_PR[3] = -half_sinR*half_sinP;
	
	q_norm = sqrtf(target_quat_PR[0]*target_quat_PR[0] + target_quat_PR[1]*target_quat_PR[1] + target_quat_PR[2]*target_quat_PR[2] + target_quat_PR[3]*target_quat_PR[3]);
	target_quat_PR[0] = target_quat_PR[0] / q_norm;
	target_quat_PR[1] = target_quat_PR[1] / q_norm;
	target_quat_PR[2] = target_quat_PR[2] / q_norm;
	target_quat_PR[3] = target_quat_PR[3] / q_norm;
	
	//���������Ԫ��
	float q_error[4];
	
	//����ϵ
//	q_error[0] = current_quat_PR[0]*target_quat_PR[0] + current_quat_PR[1]*target_quat_PR[1] + current_quat_PR[2]*target_quat_PR[2] + current_quat_PR[3]*target_quat_PR[3];
//	q_error[1] = current_quat_PR[0]*target_quat_PR[1] - current_quat_PR[1]*target_quat_PR[0] - current_quat_PR[2]*target_quat_PR[3] + current_quat_PR[3]*target_quat_PR[2];
//	q_error[2] = current_quat_PR[0]*target_quat_PR[2] - current_quat_PR[2]*target_quat_PR[0] + current_quat_PR[1]*target_quat_PR[3] - current_quat_PR[3]*target_quat_PR[1];
//	q_error[3] = current_quat_PR[0]*target_quat_PR[3] - current_quat_PR[1]*target_quat_PR[2] + current_quat_PR[2]*target_quat_PR[1] - current_quat_PR[3]*target_quat_PR[0];
	
	//����ϵ
	q_error[0] = current_quat_PR[0]*target_quat_PR[0] + current_quat_PR[1]*target_quat_PR[1] + current_quat_PR[2]*target_quat_PR[2] + current_quat_PR[3]*target_quat_PR[3];
	q_error[1] = current_quat_PR[0]*target_quat_PR[1] - current_quat_PR[1]*target_quat_PR[0] + current_quat_PR[2]*target_quat_PR[3] - current_quat_PR[3]*target_quat_PR[2];
	q_error[2] = current_quat_PR[0]*target_quat_PR[2] - current_quat_PR[2]*target_quat_PR[0] - current_quat_PR[1]*target_quat_PR[3] + current_quat_PR[3]*target_quat_PR[1];
	q_error[3] = current_quat_PR[0]*target_quat_PR[3] + current_quat_PR[1]*target_quat_PR[2] - current_quat_PR[2]*target_quat_PR[1] - current_quat_PR[3]*target_quat_PR[0];
	
	q_norm = sqrtf(q_error[0]*q_error[0] + q_error[1]*q_error[1] + q_error[2]*q_error[2] + q_error[3]*q_error[3]);
	q_error[0] = q_error[0] / q_norm;
	q_error[1] = q_error[1] / q_norm;
	q_error[2] = q_error[2] / q_norm;
	q_error[3] = q_error[3] / q_norm;
	
	//���������ת����
	float PR_rotation[3];
	float theta = 2.0f* acosf( q_error[0] );
	if(theta > PI)
		theta -= 2.0f*PI;
	float sin_half_theta = sqrtf( 1.0f - q_error[0]*q_error[0] );
//	float scale = theta / sin_half_theta;
	float scale;
	if (fabsf(sin_half_theta) < FLT_EPSILON)
		scale = 2.0f;
	else
		scale = theta / sin_half_theta;
	
	PR_rotation[0] = q_error[1] * scale * RAD2DEG;
	PR_rotation[1] = q_error[2] * scale * RAD2DEG;
	PR_rotation[2] = q_error[3] * scale * RAD2DEG;
	
//	float target_angular_rate[3];
//	target_angular_rate[0] = PR_rotation[0] * Ps * RAD2DEG;
//	target_angular_rate[1] = PR_rotation[1] * Ps * RAD2DEG;
//	target_angular_rate[2] = PR_rotation[2] * Ps * RAD2DEG;
//	
//	ctrl_angular_velocity(target_angular_rate[0],target_angular_rate[1],target_angular_rate[2],GimbalGyro_x,GimbalGyro_y,GimbalGyro_z);
	
	//PID����
	PID_run(&Roll_Angel_PID, PR_rotation[0]);
	PID_run(&Pitch_Angel_PID, PR_rotation[1]);
	PID_run(&Yaw_Angel_PID, PR_rotation[2]);
	
	//ƫ������
	if ( Yaw_Control_Enabled == true )
	{
		float angle_error = (target_Yaw - yaw_by_encoder) * RAD2DEG;
//		float angle_error = (target_Yaw - yaw_encoder) * RAD2DEG;
		PID_run(&Yaw_Angel_e_PID, angle_error);
	}
	
//	Yaw_Angel_PID.PID_Out = 0.0f;
	
	float target_angular_rate_ENU[3] = {0.0f,0.0f,0.0f};
	target_angular_rate_ENU[0] = Roll_Angel_PID.PID_Out;
	target_angular_rate_ENU[1] = Pitch_Angel_PID.PID_Out;
	target_angular_rate_ENU[2] = Yaw_Angel_PID.PID_Out + Yaw_Angel_e_PID.PID_Out;
	
	target_angular_rate_body[0] = Rotation_Matrix[0][0]*target_angular_rate_ENU[0] + Rotation_Matrix[0][1]*target_angular_rate_ENU[1] + Rotation_Matrix[0][2]*target_angular_rate_ENU[2];
	target_angular_rate_body[1] = Rotation_Matrix[1][0]*target_angular_rate_ENU[0] + Rotation_Matrix[1][1]*target_angular_rate_ENU[1] + Rotation_Matrix[1][2]*target_angular_rate_ENU[2];
	target_angular_rate_body[2] = Rotation_Matrix[2][0]*target_angular_rate_ENU[0] + Rotation_Matrix[2][1]*target_angular_rate_ENU[1] + Rotation_Matrix[2][2]*target_angular_rate_ENU[2];
}

/****************************************************************************************
*ԭ  �ͣ�void ctrl_Attitude_Q(vector3* u, Quaternion target_quat, Quaternion current_quat)
*��  �ܣ���̬����
*��  �룺*u :��̬���������
*��  �룺target_quat :Ŀ����Ԫ��
*��  �룺current_quat����ǰ��Ԫ��
*��  ������
*****************************************************************************************/
void ctrl_Attitude_Q(vector3* u, Quaternion target_quat, Quaternion current_quat)
{
//	float current_quat[4];
//	current_quat[0] = q[0];current_quat[1] = -q[1];current_quat[2] = -q[2];current_quat[3] = -q[3];
	conjugate( &current_quat );
	
	Matrix33 Rotation_Matrix;
	Quat2Mat( &Rotation_Matrix, current_quat );
//	current_quat[1] = -current_quat[1];current_quat[2] = -current_quat[2];current_quat[3] = -current_quat[3];
//	conjugate( &current_quat );
	
	//���������Ԫ��
	Quaternion q_e;

	//����ϵ���
//	Quat_Prod( &q_e, target_quat, current_quat );
	
	//����ϵ���
	Quat_Prod( &q_e, current_quat, target_quat );

	vector3 v_e;
	Quat2Axis( &v_e, q_e );
	
	v_e.x = v_e.x * RAD2DEG;
	v_e.y = v_e.y * RAD2DEG;
	v_e.z = v_e.z * RAD2DEG;
	
	//PID����
	PID_run(&Roll_Angel_PID, v_e.x);
	PID_run(&Pitch_Angel_PID, v_e.y);
	PID_run(&Yaw_Angel_PID, v_e.z);
	
	//����ϵ������bug
//	vector3 target_angular_rate_w = {0.0f,0.0f,0.0f};
//	target_angular_rate_w.x = Roll_Angel_PID.PID_Out;
//	target_angular_rate_w.y = Pitch_Angel_PID.PID_Out;
//	target_angular_rate_w.z = Yaw_Angel_PID.PID_Out;
//	
//	rotate( u, target_angular_rate_w, Rotation_Matrix);

	u->x = Roll_Angel_PID.PID_Out;
	u->y = Pitch_Angel_PID.PID_Out;
	u->z = Yaw_Angel_PID.PID_Out;
}

//void ctrl_encoder(void)
//{
//	PID_run_FloatspdVolt(&Roll_Encoder_PID, target_Roll, roll_encoder*RAD2DEG);
//	PID_run_FloatspdVolt(&Pitch_Encoder_PID, target_Pitch, pitch_encoder*RAD2DEG);
//	PID_run_FloatspdVolt(&Yaw_Encoder_PID, target_Yaw, yaw_encoder*RAD2DEG);
//}

/****************************************************************************************
*ԭ  �ͣ�void ctrl_Attitude_decoup(vector3* u, Quaternion target_quat, Quaternion current_quat)
*��  �ܣ���̬���ƣ���̬���
*��  �룺*u :��̬���������
*��  �룺target_quat :Ŀ����Ԫ��
*��  �룺current_quat����ǰ��Ԫ��
*��  ������
*****************************************************************************************/
void ctrl_Attitude_decoup(Quaternion target_quat, Quaternion current_quat, vector3 gyro, vector3 encoder)
{
	conjugate( &current_quat );
	
	//���������Ԫ��
	Quaternion q_e;

	//����ϵ���
//	Quat_Prod( &q_e, target_quat, current_quat );
	
	//����ϵ���
	Quat_Prod( &q_e, current_quat, target_quat );

	vector3 v_e;
	Quat2Axis( &v_e, q_e );
	
	v_e.x = v_e.x * RAD2DEG;
	v_e.y = v_e.y * RAD2DEG;
	v_e.z = v_e.z * RAD2DEG;
	
	//��̬����
	vector3 dq_m = {0.0f,0.0f,0.0f};
	Body2Motor(&dq_m, v_e, encoder);
	
	//PID�Ƕȿ���
	PID_run_FloatspdVolt(&Roll_Angel_PID,dq_m.x,0.0f);
	PID_run_FloatspdVolt(&Pitch_Angel_PID,dq_m.y,0.0f);
	PID_run_FloatspdVolt(&Yaw_Angel_PID,dq_m.z,0.0f);
	
	//PID���ٶȿ���
	vector3 gyro_m;
 	Body2Motor(&gyro_m, gyro, encoder);
	
	PID_run_FloatspdVolt(&Roll_Speed_PID,Roll_Angel_PID.PID_Out,gyro_m.x);
	PID_run_FloatspdVolt(&Pitch_Speed_PID,Pitch_Angel_PID.PID_Out,gyro_m.y);
	PID_run_FloatspdVolt(&Yaw_Speed_PID,Yaw_Angel_PID.PID_Out,gyro_m.z);
}
