#include "app.h"
#include <stdbool.h>
#include "math_common.h"
#include "ESKF.h"

u8 Get_USART3_Buff[32];

Send_Motor_t Send_Motor;
Get_Motor_t Get_Motor;
Fd_InsData_t Fd_InsData;

u8 Fd_InsData_State = 0;
u8 Tick10ms = 0;
u8 ahrs_count = 0;
u8 kalman_step = 0;
u32 system_time = 0;
u8 time_bit = 0;
u32 debug_count = 0;

void SendMotor(u8 CMD)
{
	uint8_t *pcs = (uint8_t *)&Send_Motor;
	uint8_t Check_c;
	Send_Motor.check_sum = 0;
	Send_Motor.head_a = 0x6a;
	Send_Motor.head_b = 0x6c;
	Send_Motor.cmd = CMD;
	Send_Motor.roll = Roll_Speed_PID.PID_Out;
	Send_Motor.yaw  =  Yaw_Speed_PID.PID_Out;
	Send_Motor.rev1 = debug_count;
	for(Check_c=0;Check_c<(sizeof(Send_Motor)-1);Check_c++)
	{
		Send_Motor.check_sum += pcs[Check_c];
	}
	UART3_SendDataDMA(pcs,sizeof(Send_Motor));
}

//正常控制电机 Vd = 0.0f, 校准电机Vq = 0.0f
u16 TimeOut_Count = 0;

void api_on_data_returned(int32_t offset, int32_t len, uint8_t * data)
{
	memcpy(&Fd_InsData.RollRate,data,len);							
	Fd_InsData_State = 2;
}

void can_receive(void)
{	
	u16 FC_len,FC_i;
	uint8_t ch = 0;
	FC_len = can_serial_available();
	for(FC_i = 0; FC_i < FC_len; FC_i++)
	{
		ch = can_serial_read_char();
		api_port_received(1, &ch);		  
	}
	FC_len = 0;
}

void api_port_send(int32_t len, uint8_t * data)
{
	can_serial_write(data,len);
}

void Task10ms(void)
{
  if(Tick10ms >= 10) 
  {
    Tick10ms = 0;	
		api_loop(system_time);
		can_transmit();
		if(Fd_InsData_State == 1)
		{
		  api_auto_load_data(0,18,1,12);
		}
  }
}

float roll_encoder_offset = -1.0f,pitch_encoder_offset = -48.0f,yaw_encoder_offset = -10.0f;
float roll_dir = 1.0f,pitch_dir = -1.0f,yaw_dir = 1.0f;
float pitch_encoder = 0.0f, roll_encoder = 0.0f, yaw_encoder = 0.0f;//编码器输出的角度（弧度制）
float pitch_by_encoder = 0.0f, roll_by_encoder = 0.0f, yaw_by_encoder = 0.0f;

/****************************************************************************************
*原  型：void cal_encoder_angle(vector3* encoder)
*功  能：获取编码器角度
*输  入：*encoder :编码器结构体变量，单位rad
*输  出：无
*****************************************************************************************/
void cal_encoder_angle(vector3* encoder)
{
	float temp_roll_encoder = Get_Encoder.Angle_R + roll_encoder_offset;
	if( temp_roll_encoder > 180 )
	{
		temp_roll_encoder = temp_roll_encoder - 360;
	}
	encoder->x = roll_dir * temp_roll_encoder * DEG2RAD;
	
	float temp_pitch_encoder = Get_Encoder.Angle_P + pitch_encoder_offset;
	if( temp_pitch_encoder > 180 )
	{
		temp_pitch_encoder = temp_pitch_encoder - 360;
	}
	encoder->y = pitch_dir * temp_pitch_encoder * DEG2RAD;
//	temp_pitch_encoder = pitch_dir * temp_pitch_encoder;
//	if( temp_pitch_encoder > 90)
//	{
//		temp_pitch_encoder = 180 - temp_pitch_encoder;
//	}
//	pitch_encoder = temp_pitch_encoder * DEG2RAD;
	
	float temp_yaw_encoder = Get_Encoder.Angle_Y + yaw_encoder_offset;
	if( temp_yaw_encoder > 180)
	{
		temp_yaw_encoder = temp_yaw_encoder - 360;
	}
	encoder->z = yaw_dir * temp_yaw_encoder * DEG2RAD;
	
//	float half_sinR, half_cosR;
//	float half_sinP, half_cosP;
//	float half_sinY, half_cosY;
//	half_sinR = sinf(0.5f*roll_encoder); half_cosR = cosf(0.5f*roll_encoder);
//	half_sinP = sinf(0.5f*pitch_encoder); half_cosP = cosf(0.5f*pitch_encoder);
//	half_sinY = sinf(0.5f*yaw_encoder); half_cosY = cosf(0.5f*yaw_encoder);
//	
//	float q[4] = {1.0f,0.0f,0.0f,0.0f};
//	q[0] = half_cosR*half_cosP*half_cosY - half_sinR*half_sinP*half_sinY;
//	q[1] = half_sinR*half_cosP*half_cosY - half_cosR*half_sinP*half_sinY;
//	q[2] = half_cosR*half_sinP*half_cosY + half_sinR*half_cosP*half_sinY;
//	q[3] = half_cosR*half_cosP*half_sinY + half_sinR*half_sinP*half_cosY;
//	
//	float q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
//	q[0] = q[0] / q_norm;
//	q[1] = q[1] / q_norm;
//	q[2] = q[2] / q_norm;
//	q[3] = q[3] / q_norm;
//	
//	roll_by_encoder = atan2f( 2.0f*(q[0]*q[1]+q[2]*q[3]) , 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]) );
//	pitch_by_encoder = asinf( 2.0f*(q[0]*q[2]-q[1]*q[3]) );
//	yaw_by_encoder = atan2f( 2.0f*(q[0]*q[3]+q[1]*q[2]) , 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]) );

//	if( pitch_encoder * RAD2DEG < 80 )
//	{
//		yaw_by_encoder = yaw_encoder;
//	}
//	else
//	{
//		yaw_by_encoder = roll_encoder;
//	}

	float cosp = cosf(pitch_encoder * DEG2RAD);
    float sinp = sinf(pitch_encoder * DEG2RAD);
	float cosr = cosf(roll_encoder * DEG2RAD);
       
    yaw_by_encoder = (yaw_encoder * cosp * cosr + roll_encoder * sinp) * DEG2RAD;
}

u8 cmd_value = 0;//0 - 正常运行 1 - 校准
u8 control_mode = 0;//0 - 姿态控制模式 1 - 编码器控制模式
bool calib_gyroscope = true;
vector3 current_angular_rate;//减去零偏的角速度
Quaternion target_quat;//目标四元数
void Gimbal_Control(void)
{	
	uint8_t *pc = (uint8_t *)&Send_Motor;
	uint8_t Check_i;
	
	if(Selection_axis == 0)//pitch 轴
	{
		if(U3_IDLE_Flag == 1)
		{
			U3_IDLE_Flag = 0;	
			if(Get_USART3_Buff[12] == 0x6a && Get_USART3_Buff[13] == 0x6c)
			{
				memcpy(&Get_Motor,Get_USART3_Buff,sizeof(Get_Motor));					
				Get_Motor.check_sum = 0;
				for(Check_i=0;Check_i<(sizeof(Get_Motor)-1);Check_i++)
				{
					Get_Motor.check_sum += Get_USART3_Buff[Check_i];
				}
				
				if(Get_Motor.check_sum == Get_USART3_Buff[sizeof(Get_Motor)-1])
				{
					if(Get_Motor.cmd == 0x00)
					{
						Get_Encoder.Angle_R = Get_Motor.roll;
						Get_Encoder.Angle_Y = Get_Motor.yaw;
					}
				}
				memset(Get_USART3_Buff,0,sizeof(char)*16);
			}
		}			
    
		can_receive();
		Task10ms();
		
		if(TIM4_Flag == 1 && MPU_Flag == 1)//0.5ms执行一次
		{			
			if(++time_bit >= 2)//1ms计数
			{
				time_bit = 0;
				Tick10ms++;
				ahrs_count++;
				system_time++;
			}
//			GPIO_ResetBits(GPIOB,GPIO_Pin_6);	
			
			// 获取IMU数据
			IMU_Update(&gyro,&acc);//50us
			// 获取编码器角度
			cal_encoder_angle(&Encoder);//20us
			// 获取飞控角度
			Get_Vehicle_Attitude(&Angle_Vehicle,Fd_InsData);
			// 加速度滤波
			Filter_LP_IIR_1(&acc_filted, acc);
//			if( calib_gyroscope == true )
//			{
//				cal_gyro_bias();
////				inited = true;
//			}
			if( AHRS_OK == true )
			{	
				// 使用加速度进行姿态解算
//				MS_Attitude_Acconly(acc, &roll_acc, &pitch_acc);//10us
				MS_Attitude_Acconly(acc_filted, &roll_acc_filted, &pitch_acc_filted);//10us
				
//			MS_Attitude_GyroIntegral();
				// 使用CF进行姿态解算
//				MS_Attitude_Mahony();//30us
//				MS_Attitude_CF_Bias(&qEst,&bias,gyro,acc);
//				MS_Attitude_CF_Bias_FC(&qEst,&bias,gyro,Angle_Vehicle,Encoder);
				
				// 使用KF进行姿态解算
				MS_Attitude_GyroIntegral(&qEst, gyro, bias, dt);			
				if( ahrs_count >= 1 )//每1ms执行一次
				{
					ahrs_count = 0;
					MS_Attitude_ESKF_Bias_FC(&qEst,&bias,gyro,Angle_Vehicle,Encoder,&kalman_step);
				}
				
//				Quat2Euler( &euler_Est, qEst );
				Quat2Euler_G( &euler_Est, qEst );
			
//			if (control_mode == 0)
//			{
				//计算目标姿态
//				float half_sinYaw, half_cosYaw;
//				half_sinYaw = sinf(Angle_Vehicle.z*0.5f);
//				half_cosYaw = cosf(Angle_Vehicle.z*0.5f);
//				target_quat.qw = half_cosYaw;
//				target_quat.qz = half_sinYaw;
			
//				float pitch_max;
//				pitch_max = target_attitude_range_pitch(-40.0f*DEG2RAD, Angle_Vehicle, Encoder);
				
				float roll = 0.0f;
				float pitch = 0.0f;
				float yaw_head = 0.0f;
				vector3 target_attitude;
//				target_attitude.x = 0.0f * DEG2RAD;
//				target_attitude.y = 0.0f * DEG2RAD;
//				target_attitude.z = Angle_Vehicle.z + yaw_head*DEG2RAD;
				
				set_target_attitude(&target_attitude, roll, pitch, yaw_head);
				
				//目标姿态保护
//				target_attitude_range1(&target_attitude);
//				target_attitude_range_rp(&target_attitude, Angle_Vehicle, Encoder, qEst);
//				target_attitude_range3(&target_attitude, Angle_Vehicle, yaw_head, qEst);
//				target_attitude_range5(&target_attitude, Angle_Vehicle, yaw_head*DEG2RAD, Encoder);
//				target_attitude_range6(&target_attitude, Angle_Vehicle, yaw_head, qEst, Encoder);
				
				debug_buf[0] = target_attitude.x*RAD2DEG;
				debug_buf[1] = target_attitude.y*RAD2DEG;
				debug_buf[2] = target_attitude.z*RAD2DEG;
				
//				Euler2Quat( &target_quat, target_attitude.x, target_attitude.y, target_attitude.z);
				Euler2Quat_G( &target_quat, target_attitude.x, target_attitude.y, target_attitude.z);
				
//				target_attitude_range4(&target_quat,target_attitude,Angle_Vehicle,Encoder);
				
				current_angular_rate.x = (gyro.x + bias.x)*RAD2DEG;
				current_angular_rate.y = (gyro.y + bias.y)*RAD2DEG;
				current_angular_rate.z = (gyro.z + bias.z)*RAD2DEG;

				// 姿态环
				ctrl_Attitude_Q(&target_angular_rate,target_quat,qEst);
				// 角速度环
				ctrl_angular_velocity(target_angular_rate,current_angular_rate);//15us
				
				// 双解耦控制策略
//				ctrl_Attitude_decoup(target_quat, qEst, current_angular_rate, Encoder);
//			}
			}
			else
			{
				init_MS_Attitude(&qEst);
			}
//			else if (control_mode == 1)
//			{
//				ctrl_encoder();
//				Pitch_Speed_PID.PID_Out = Pitch_Encoder_PID.PID_Out;
//				Roll_Speed_PID.PID_Out = Roll_Encoder_PID.PID_Out;
//				Yaw_Speed_PID.PID_Out = Yaw_Encoder_PID.PID_Out;
//			}
			
//			GPIO_SetBits(GPIOB,GPIO_Pin_6);	
			
////			PID_run_FloatspdVolt(&Pitch_Angel_PID,0.0f,pitch);//角度环
//			PID_run_FloatspdVolt(&Pitch_Speed_PID,0.0f,GimbalGyro_y);//角速度环
//			
////			PID_run_FloatspdVolt(&Roll_Angel_PID ,0.0f,roll);
//			PID_run_FloatspdVolt(&Roll_Speed_PID,0.0f,GimbalGyro_x);
//						
////			PID_run_FloatspdVolt(&Yaw_Angel_PID,0.0f,yaw);									
//			PID_run_FloatspdVolt(&Yaw_Speed_PID,0.0f,GimbalGyro_z);
			
			// 控制输出
//			Pitch_Speed_PID.PID_Out = 0.0f;
//			Roll_Speed_PID.PID_Out = 0.0f;
//			Yaw_Speed_PID.PID_Out = 0.0f;
			
			if(Pitch_Speed_PID.PID_Out > 0.5f)Pitch_Speed_PID.PID_Out = 0.5f; 
			else if(Pitch_Speed_PID.PID_Out < -0.5f)Pitch_Speed_PID.PID_Out = -0.5f;
      
			if(Roll_Speed_PID.PID_Out > 0.5f)Roll_Speed_PID.PID_Out = 0.5f; 
			else if(Roll_Speed_PID.PID_Out < -0.5f)Roll_Speed_PID.PID_Out = -0.5f;
			
			if(Yaw_Speed_PID.PID_Out > 0.3f)Yaw_Speed_PID.PID_Out = 0.3f; 
			else if(Yaw_Speed_PID.PID_Out < -0.3f)Yaw_Speed_PID.PID_Out = -0.3f;
			
			if (cmd_value == 0)
			{
				MotorOut_PR(Get_Encoder.Angle_P*MotorPR_Radian,-Pitch_Speed_PID.PID_Out,0.0f);
			}
			else if (cmd_value == 1)
			{
				MotorOut_PR(0.0f,0.0f,0.3f);
			}
			
			SendMotor(cmd_value);
			Oscilloscope();//数据打印			
			R_LED_Status(3);
			G_LED_Status(4);
			TIM4_Flag = 0;
			MPU_Flag = 0;
		}
	}
	else if(Selection_axis == 1)// roll yaw 轴
	{
		if(U3_IDLE_Flag == 1)
		{
			U3_IDLE_Flag = 0;			
			TimeOut_Count = 0;
			
			if(Get_USART3_Buff[12] == 0x6a && Get_USART3_Buff[13] == 0x6c)
			{
				memcpy(&Get_Motor,Get_USART3_Buff,sizeof(Get_Motor));
				
				Get_Motor.check_sum = 0;
				for(Check_i=0;Check_i<(sizeof(Get_Motor)-1);Check_i++)
				{
					Get_Motor.check_sum += Get_USART3_Buff[Check_i];
				}				
				if(Get_Motor.check_sum == Get_USART3_Buff[sizeof(Get_Motor)-1])
				{
					if(Get_Motor.cmd == 0x00)//正常运行
					{
						MotorOut_PR(Get_Encoder.Angle_R*MotorPR_Radian,Get_Motor.roll,0.0f);// Angle,Vq <= 0.5f,校准 Vd = 0.3f
		                MotorOut_Y(Get_Encoder.Angle_Y*MotorY_Radian,Get_Motor.yaw,0.0f);// Angle,Vq <= 0.3f,校准 Vd = 0.15f
					}
					else if(Get_Motor.cmd == 0x01)//校准
					{
						MotorOut_PR(0.0f,0.0f,0.3f);
						MotorOut_Y(0.0f,0.0f,0.15f);
					}
				}				
			}
			Send_Motor.check_sum = 0;
			Send_Motor.head_a = 0x6a;
			Send_Motor.head_b = 0x6c;
			Send_Motor.cmd = Get_Motor.cmd;
			Send_Motor.rev1 = debug_count;
			if(Get_Motor.cmd == 0x0)
			{
				Send_Motor.roll = Get_Encoder.Angle_R;
				Send_Motor.yaw  = Get_Encoder.Angle_Y;
			}
			else
			{
				Send_Motor.roll = duty_data;
				Send_Motor.yaw  = duty_data_Y;
			}
			for(Check_i=0;Check_i<(sizeof(Send_Motor)-1);Check_i++)
			{
				Send_Motor.check_sum += pc[Check_i];
			}
			UART3_SendDataDMA(pc,sizeof(Send_Motor));
				
			memset(Get_USART3_Buff,0,sizeof(char)*16);
		}
		else
		{
			delay_us(1);
			
			if(TimeOut_Count < 2000)//2ms收不到数据关闭电机
			{
				TimeOut_Count++;
				if(TimeOut_Count >= 2000)
				{
					TimeOut_Count = 2000;
					debug_count++;
					MotorOut_PR(0.0f,0.0f,0.0f);// Angle,Vq <= 0.5f,校准 Vd = 0.3f
					MotorOut_Y(0.0f,0.0f,0.0f);// Angle,Vq <= 0.3f,校准 Vd = 0.15f
				}
			}
		}
	}
}

