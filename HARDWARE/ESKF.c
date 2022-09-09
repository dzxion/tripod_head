#include "math_common.h"
#include "sys.h"
#include "AHRS.h"
#include "app.h"

const float dt = 0.0005f; 
float KF_F[36] = { 1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                   0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,
                   0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,
                   0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
                   0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,
                   0.0f,0.0f,0.0f,0.0f,0.0f,1.0f};//状态传递矩阵
float KF_X_pri[6]  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//状态先验估计
float KF_Xk[6]  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//状态变量
float KF_Pk[36] = { 100.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
                        0.0f,100.0f,    0.0f,    0.0f,    0.0f,    0.0f,
                        0.0f,    0.0f,100.0f,    0.0f,    0.0f,    0.0f,
                        0.0f,    0.0f,    0.0f,100.0f,    0.0f,    0.0f,
                        0.0f,    0.0f,    0.0f,    0.0f,100.0f,    0.0f,
                        0.0f,    0.0f,    0.0f,    0.0f,    0.0f,100.0f};//状态协方差矩阵
float KF_P_1[36] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//计算中间矩阵
float F_T[36];//运算中间矩阵
float KF_P_pri[36] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//协方差先验估计
float KF_Q[36]  = { 0.0075f*dt*10.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0075f*dt*10.0f,0.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0075f*dt*10.0f,0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0003f*dt*10.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0003f*dt*10.0f,0.0f,
                    0.0f,0.0f,0.0f,0.0f,0.0f,0.0003f*dt*10.0f};//过程噪声协方差矩阵
float KF_P_xr[] = { 0.0f,0.0f,0.0f,
                    0.0f,0.0f,0.0f, 
                    0.0f,0.0f,0.0f};//运算中间矩阵
float KF_R[9]   = {0.075f,0.0f,0.0f,
                   0.0f,0.075f,0.0f,
	               0.0f,0.0f,0.075f};//测量噪声协方差矩阵
float KF_P_xr_inc[] = { 0.0f,0.0f,0.0f,
                        0.0f,0.0f,0.0f,
                        0.0f,0.0f,0.0f};//运算中间矩阵
float KF_K[18] = { 0.0f,0.0f,0.0f,
	                 0.0f,0.0f,0.0f,
                   0.0f,0.0f,0.0f,
                   0.0f,0.0f,0.0f,
	                 0.0f,0.0f,0.0f,
                   0.0f,0.0f,0.0f};//增益K矩阵
float KF_Zk[3] = {0.0f,0.0f,0.0f};//测量值
float KF_dZk[3] = {0.0f,0.0f,0.0f};//运算中间矩阵
float KF_dXk[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//运算中间矩阵
float KF_w[3] = {0.0f,0.0f,0.0f};//零偏
				   
void KF_Set_F(Quaternion q)
{
	Matrix33 Rwb;
	Quat2Mat(&Rwb,q);
	KF_F[3] = -Rwb.a11*dt*10.0f;
	KF_F[4] = -Rwb.a12*dt*10.0f;
	KF_F[5] = -Rwb.a13*dt*10.0f;
	KF_F[9] = -Rwb.a21*dt*10.0f;
	KF_F[10] = -Rwb.a22*dt*10.0f;
	KF_F[11] = -Rwb.a23*dt*10.0f;
	KF_F[15] = -Rwb.a31*dt*10.0f;
	KF_F[16] = -Rwb.a32*dt*10.0f;
	KF_F[17] = -Rwb.a33*dt*10.0f;
//	KF_F[21] = 1.0f-1.0f/tao*ts*5.0f;
//	KF_F[28] = 1.0f-1.0f/tao*ts*5.0f;
//	KF_F[35] = 1.0f-1.0f/tao*ts*5.0f;	
}

/****************************************************************************************
*原  型：void KF_FeedBack(void)
*功  能：使用观测误差更新原状态
*输  入：*q 估计的姿态四元数
*输  入：*bias 估计的陀螺零偏 rad/s
*****************************************************************************************/
void KF_FeedBack(Quaternion* q, vector3* bias)
{
	//姿态补偿
	Quaternion dQ;
	vector3 corr = {KF_Xk[0],KF_Xk[1],KF_Xk[2]};
	Axis2Quat(&dQ,corr);
	Quat_Prod(q,dQ,*q);
	
	//零偏补偿
	KF_w[0] += KF_Xk[3];
	KF_w[1] += KF_Xk[4];
	KF_w[2] += KF_Xk[5];
		
	//Integral limiter
	if( KF_w[0] > bias_max)
	{
		KF_w[0] = bias_max;
	}		
	if( KF_w[0] < -bias_max)
	{
		KF_w[0] = -bias_max;
	}
	
	if( KF_w[1] > bias_max)
	{
		KF_w[1] = bias_max;
	}
	if( KF_w[1] < -bias_max)
	{
		KF_w[1] = -bias_max;
	}
	
	if( KF_w[2] > bias_max)
	{
		KF_w[2] = bias_max;
	}
	if( KF_w[2] < -bias_max)
	{
		KF_w[2] = -bias_max;
	}
	
	//误差状态重置
	KF_Xk[0] = 0.0f;
	KF_Xk[1] = 0.0f;
	KF_Xk[2] = 0.0f;
	KF_Xk[3] = 0.0f;
	KF_Xk[4] = 0.0f;
	KF_Xk[5] = 0.0f;
	
	bias->x = -KF_w[0];
	bias->y = -KF_w[1];
	bias->z = -KF_w[2];
}

/****************************************************************************************
*原  型：void MS_Attitude_ESKF_Bias_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder,u8* step)
*功  能：基于飞控姿态的姿态估计（ESFF）带零偏
*输  入：*q 估计的姿态四元数
*输  入：*bias 估计的陀螺零偏 rad/s
*输  入：gyro 陀螺角速度 rad/s
*输  入：angle_vehicle 姿态角度 rad
*输  入：encoder 编码器角度 rad
*输  入：*step 步骤
*输  出：无
*****************************************************************************************/
void MS_Attitude_ESKF_Bias_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder,u8* step)
{
	int i=0,j=0,k=0;
	float temp = 0.0f;
	switch(*step)
	{
		case 0: //Xk = F*Xk
		{
			KF_Set_F(*q);
			for(i=0;i<6;i++)
			{
				temp = 0.0f;
				for(j=0;j<6;j++)
				{
					temp += KF_F[i*6+j]*KF_Xk[j];
				}
				KF_X_pri[i] = temp;
			}
			(*step)++;
			break;
		}
		case 1://Pk_1 = F*Pk
		{
			Mat66mul(&KF_P_1[0],KF_F,KF_Pk);
			(*step)++;	
			break;			
		}
		case 2://Pk_ = Pk_1*F'+Q
		{
			Mat_T66(&F_T[0],KF_F);
			Mat66mul(&KF_P_pri[0],KF_P_1,F_T);
			for(i = 0;i<36;i++)
			{
				KF_P_pri[i] += KF_Q[i];
			}
			(*step)++;	
			break;
		}
		case 3://Pkxr = H*Pk_*H'+R
		{
			KF_P_xr[0] = KF_P_pri[0]+ KF_R[0];
			KF_P_xr[1] = KF_P_pri[1]+ KF_R[1];
			KF_P_xr[2] = KF_P_pri[2]+ KF_R[2];
			KF_P_xr[3] = KF_P_pri[6]+ KF_R[3];
			KF_P_xr[4] = KF_P_pri[7]+ KF_R[4];
			KF_P_xr[5] = KF_P_pri[8]+ KF_R[5];
			KF_P_xr[6] = KF_P_pri[12]+ KF_R[6];
			KF_P_xr[7] = KF_P_pri[13]+ KF_R[7];	
			KF_P_xr[8] = KF_P_pri[14]+ KF_R[8];
			(*step)++;	
			break;
		}
		case 4://Kk = Pk_*H'*inc(H*Pk_*H'+R)' 
		{
			temp =  KF_P_xr[0]*(KF_P_xr[4]*KF_P_xr[8]-KF_P_xr[7]*KF_P_xr[5])
      			  - KF_P_xr[1]*(KF_P_xr[3]*KF_P_xr[8]-KF_P_xr[6]*KF_P_xr[5])
      			  + KF_P_xr[2]*(KF_P_xr[3]*KF_P_xr[7]-KF_P_xr[6]*KF_P_xr[4]);
			if (temp<0.000001f)
			{
				temp = 0.000001f;
			}
			KF_P_xr_inc[0] = (KF_P_xr[4]*KF_P_xr[8]-KF_P_xr[7]*KF_P_xr[5])/temp;
			KF_P_xr_inc[1] = -(KF_P_xr[1]*KF_P_xr[8]-KF_P_xr[7]*KF_P_xr[2])/temp;
			KF_P_xr_inc[2] = (KF_P_xr[1]*KF_P_xr[5]-KF_P_xr[4]*KF_P_xr[2])/temp;
			KF_P_xr_inc[3] = -(KF_P_xr[3]*KF_P_xr[8]-KF_P_xr[6]*KF_P_xr[5])/temp;
			KF_P_xr_inc[4] = (KF_P_xr[0]*KF_P_xr[8]-KF_P_xr[6]*KF_P_xr[2])/temp;
			KF_P_xr_inc[5] = -(KF_P_xr[0]*KF_P_xr[5]-KF_P_xr[3]*KF_P_xr[2])/temp;
			KF_P_xr_inc[6] = (KF_P_xr[3]*KF_P_xr[7]-KF_P_xr[6]*KF_P_xr[4])/temp;
			KF_P_xr_inc[7] = -(KF_P_xr[0]*KF_P_xr[7]-KF_P_xr[6]*KF_P_xr[1])/temp;
			KF_P_xr_inc[8] = (KF_P_xr[0]*KF_P_xr[4]-KF_P_xr[3]*KF_P_xr[1])/temp;
			(*step)++;	
			break;
		}
		case 5://Kk = Pk_*H'*inc
		{
			for(i=0;i<6;i++)
			{		
				for(j=0;j<3;j++)
				{
					temp = 0.0f;
					for(k=0;k<3;k++)
					{
						temp += KF_P_pri[i*6+k]*KF_P_xr_inc[k*3+j];
					}
					KF_K[i*3+j]= temp;
				}
			}
			(*step)++;	
			break;
		}
		case 6://Pk = (I-KH)*P_
		{
			for(i=0;i<36;i++)
			{		
				KF_Pk[i]= KF_P_pri[i];				
			}	
			
			for(i=0;i<6;i++)
			{		
				for(j=0;j<6;j++)
				{
					temp = 0.0f;
					for(k = 0;k<3;k++)
					{
						temp += KF_K[i*3+k]*KF_P_pri[k*6+j];
					}
					KF_Pk[i*6+j] -= temp;
				}
			}
			(*step)++;	
			break;
		}
		case 7: //Xk = Xk_[i];+Kk*(Zk-H*Xk_) 
		{
			//计算飞控姿态四元数
			Quaternion quat_fc;
			Euler2Quat( &quat_fc, angle_vehicle.x, angle_vehicle.y, angle_vehicle.z );
			
			//获取编码器角度并计算四元数
			Quaternion quat_encoder;
			Get_Encoder_Quat( &quat_encoder, encoder.x, encoder.y, encoder.z);
			
			//结合飞控姿态计算相机姿态
			Quaternion quat_measure;
			Quat_Prod( &quat_measure, quat_fc, quat_encoder );
			
			//求误差四元数
			Quaternion current_quat_conj = *q;
			conjugate(&current_quat_conj);
			Quaternion quat_error;
			Quat_Prod( &quat_error, quat_measure, current_quat_conj );
			vector3 angle_error;
//			Quat2Euler( &angle_error, quat_error );
			Quat2Axis( &angle_error, quat_error );
			
			KF_Zk[0] =  angle_error.x;
			KF_Zk[1] =  angle_error.y;
			KF_Zk[2] =  angle_error.z;
			
			for(i=0;i<3;i++)
			{
				KF_dZk[i] = KF_Zk[i]-KF_X_pri[i];
			}
			for(i=0;i<6;i++)
			{	
				temp = 0.0f;
				for(j=0;j<3;j++)
				{
					temp+=KF_K[i*3+j]*KF_dZk[j];
				}	
				KF_dXk[i] = temp;
			}
			for(i=0;i<6;i++)
			{	
				KF_Xk[i] = KF_X_pri[i] + KF_dXk[i];
			}
			KF_FeedBack(q,bias);
			(*step) = 0;	
			break;
		}
		default :
			break;
	}
}
