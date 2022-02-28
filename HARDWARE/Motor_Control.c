#include "app.h"

u8 Get_USART3_Buff[32];

Send_Motor_t Send_Motor;
Get_Motor_t Get_Motor;
Fd_InsData_t Fd_InsData;

u8 Fd_InsData_State = 0;
u8 Tick10ms = 0;
u32 system_time = 0;
u8 time_bit = 0;

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

float pitch_encoder = 0.0f, roll_encoder = 0.0f, yaw_encoder = 0.0f;
void cal_encoder_angle(void)
{
	if( Get_Encoder.Angle_R > 180 )
	{
		roll_encoder = (Get_Encoder.Angle_R - 360) * DEG2RAD;
	}
	else
	{
		roll_encoder = Get_Encoder.Angle_R * DEG2RAD;
	}
	
	float temp_pitch_encoder = Get_Encoder.Angle_P - 45;
	if( Get_Encoder.Angle_P > 180 )
	{
		temp_pitch_encoder = temp_pitch_encoder - 360;
	}
	pitch_encoder = -temp_pitch_encoder * DEG2RAD;
		
}

u8 cmd_value = 0;//0 - 正常运行 1 - 校准
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
    
//		can_receive();
//		Task10ms();
		
		if(TIM4_Flag == 1 && MPU_Flag == 1)
		{			
			if(++time_bit >= 2)
			{
				time_bit = 0;
				Tick10ms++;
				system_time++;
			}
			
			// 估计解算
			IMU_Update();
			MS_Attitude_Acconly();
//			MS_Attitude_GyroIntegral();
			MS_Attitude_Mahony();
			
			// 控制解算
			cal_encoder_angle();
			
			ctrl_angular_velocity(0.0f,0.0f,0.0f,GimbalGyro_x,GimbalGyro_y,GimbalGyro_z);
			
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
					MotorOut_PR(0.0f,0.0f,0.0f);// Angle,Vq <= 0.5f,校准 Vd = 0.3f
					MotorOut_Y(0.0f,0.0f,0.0f);// Angle,Vq <= 0.3f,校准 Vd = 0.15f
				}
			}
		}
	}
}

