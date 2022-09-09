#include "math_common.h"
#include <math.h>
#include "arm_math.h"
#include <float.h>

/*旋转相关函数*/
void normalize( Quaternion* q )
{
	float inv_length = sqrtf( q->qw*q->qw + q->qx*q->qx + q->qy*q->qy + q->qz*q->qz );
	inv_length = 1.0f / inv_length;
	q->qw *= inv_length;
	q->qx *= inv_length;
	q->qy *= inv_length;
	q->qz *= inv_length;
}

void conjugate( Quaternion* q )
{
	q->qx = -q->qx;
	q->qy = -q->qy;
	q->qz = -q->qz;
}

void Quat_Prod( Quaternion* q, Quaternion a, Quaternion b )
{
	q->qw = a.qw*b.qw - a.qx*b.qx - a.qy*b.qy - a.qz*b.qz;
	q->qx = a.qw*b.qx + a.qx*b.qw + a.qy*b.qz - a.qz*b.qy;
	q->qy = a.qw*b.qy + a.qy*b.qw - a.qx*b.qz + a.qz*b.qx;
	q->qz = a.qw*b.qz + a.qx*b.qy - a.qy*b.qx + a.qz*b.qw;
	
	normalize(q);
}

void integral( Quaternion* q, vector3 delta_angle )
{
	float tqw=q->qw;	float tqx=q->qx;	float tqy=q->qy;	float tqz=q->qz;
	q->qw += 0.5f * ( -tqx*delta_angle.x - tqy*delta_angle.y - tqz*delta_angle.z );
	q->qx += 0.5f * ( tqw*delta_angle.x + tqy*delta_angle.z - tqz*delta_angle.y );
	q->qy += 0.5f * ( tqw*delta_angle.y - tqx*delta_angle.z + tqz*delta_angle.x );
	q->qz += 0.5f * ( tqw*delta_angle.z + tqx*delta_angle.y - tqy*delta_angle.x );
	normalize(q);
}

void Quat2Axis( vector3* vec, Quaternion q )
{
	float theta = 2.0f * acosf( q.qw );
	if(theta > PI)
		theta -= 2.0f * PI;
	float sin_half_theta = sqrtf( 1.0f - q.qw*q.qw );
	float scale;
	if( fabsf(sin_half_theta) < FLT_EPSILON )
		scale = 2.0f;
	else
		scale = theta / sin_half_theta;
	vec->x = q.qx * scale;
	vec->y = q.qy * scale;
	vec->z = q.qz * scale;
}

void Axis2Quat( Quaternion* q, vector3 vec )
{
	float theta = sqrtf(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
	vec.x = vec.x / theta;
	vec.y = vec.y / theta;
	vec.z = vec.z / theta;
	
	float half_sin, half_cos;
	half_sin = sinf(0.5f*theta); half_cos = cosf(0.5f*theta);
	
	q->qw = half_cos;
	q->qx = half_sin*vec.x;
	q->qy = half_sin*vec.y;
	q->qz = half_sin*vec.z;
}

void Euler2Quat( Quaternion* q, float roll, float pitch, float yaw )
{
	float half_sinR, half_cosR;
	float half_sinP, half_cosP;
	float half_sinY, half_cosY;
	half_sinR = sinf(0.5f*roll); half_cosR = cosf(0.5f*roll);
	half_sinP = sinf(0.5f*pitch); half_cosP = cosf(0.5f*pitch);
	half_sinY = sinf(0.5f*yaw); half_cosY = cosf(0.5f*yaw);
	
	q->qw = half_cosR*half_cosP*half_cosY + half_sinR*half_sinP*half_sinY;
	q->qx = half_sinR*half_cosP*half_cosY - half_cosR*half_sinP*half_sinY;
	q->qy = half_cosR*half_sinP*half_cosY + half_sinR*half_cosP*half_sinY;
	q->qz = half_cosR*half_cosP*half_sinY - half_sinR*half_sinP*half_cosY;
	
	normalize(q);
}

void Quat2Euler( vector3* euler, Quaternion q )
{
	euler->x = atan2f( 2.0f*(q.qw*q.qx+q.qy*q.qz) , 1.0f-2.0f*(q.qx*q.qx+q.qy*q.qy) );
	euler->y = asinf( 2.0f*(q.qw*q.qy-q.qx*q.qz) );
	euler->z = atan2f( 2.0f*(q.qw*q.qz+q.qx*q.qy) , 1.0f-2.0f*(q.qy*q.qy+q.qz*q.qz) );
}

void Euler2Quat_G( Quaternion* q, float roll, float pitch, float yaw )
{
	float half_sinR, half_cosR;
	float half_sinP, half_cosP;
	float half_sinY, half_cosY;
	half_sinR = sinf(0.5f*roll); half_cosR = cosf(0.5f*roll);
	half_sinP = sinf(0.5f*pitch); half_cosP = cosf(0.5f*pitch);
	half_sinY = sinf(0.5f*yaw); half_cosY = cosf(0.5f*yaw);
	
	q->qw = half_cosR*half_cosP*half_cosY - half_sinR*half_sinP*half_sinY;
	q->qx = half_sinR*half_cosP*half_cosY - half_cosR*half_sinP*half_sinY;
	q->qy = half_cosR*half_sinP*half_cosY + half_sinR*half_cosP*half_sinY;
	q->qz = half_cosR*half_cosP*half_sinY + half_sinR*half_sinP*half_cosY;
	
	normalize(q);
}

void Quat2Euler_G( vector3* euler, Quaternion q )
{
	euler->x = asinf( 2.0f*(q.qw*q.qx+q.qy*q.qz) );
	euler->y = atan2f( 2.0f*(q.qw*q.qy-q.qx*q.qz) , 1.0f-2.0f*(q.qx*q.qx+q.qy*q.qy) );
	euler->z = atan2f( 2.0f*(q.qw*q.qz-q.qx*q.qy) , 1.0f-2.0f*(q.qx*q.qx+q.qz*q.qz) );
}

void Quat2Mat( Matrix33* mat, Quaternion q )
{
	float qw2 = q.qw * q.qw;
	float qx2 = q.qx * q.qx;
	float qy2 = q.qy * q.qy;
	float qz2 = q.qz * q.qz;
	float qwx = q.qw * q.qx;
	float qwy = q.qw * q.qy;
	float qwz = q.qw * q.qz;
	float qxy = q.qx * q.qy;
	float qxz = q.qx * q.qz;
	float qyz = q.qy * q.qz;
	
	mat->a11=qw2+qx2-qy2-qz2;	mat->a12=2.0f*(qxy-qwz);	mat->a13=2.0f*(qwy+qxz);
	mat->a21=2.0f*(qwz+qxy);	mat->a22=qw2-qx2+qy2-qz2;	mat->a23=2.0f*(qyz-qwx);
	mat->a31=2.0f*(qxz-qwy);	mat->a32=2.0f*(qwx+qyz);	mat->a33=qw2-qx2-qy2+qz2;
}

void rotate( vector3* vec_out, vector3 vec_in, Matrix33 mat )
{
	vec_out->x = mat.a11*vec_in.x + mat.a12*vec_in.y + mat.a13*vec_in.z;
	vec_out->y = mat.a21*vec_in.x + mat.a22*vec_in.y + mat.a23*vec_in.z;
	vec_out->z = mat.a31*vec_in.x + mat.a32*vec_in.y + mat.a33*vec_in.z;
}
/*旋转相关函数*/

/*矩阵相关函数*/
/***********************************************************************************************************
*原型:void Mat_T66(float*C,float A[36])
*功能：6维方阵转置
*调用：
*输入： 
*输出：
************************************************************************************************************/
void Mat_T66(float*C,float A[36])
{
	u8 i,j;
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j++)
		{
			*(C+i*6+j) = A[i+j*6];		
		}
	}
}

/***********************************************************************************************************
*原型:void Mat66mul(float*C,float A[18],float B[18])
*功能：6维方阵乘法
*调用：
*输入： 
*输出：
************************************************************************************************************/
void Mat66mul(float*C,float A[36],float B[36])
{
	u8 i,j,k;
	float temp;
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j++)
		{
			temp = 0.0f;
			for(k=0;k<6;k++)
			{
				temp += A[i*6+k]*B[j+k*6];
			}
			*(C+(i*6+j)) = temp;
		}
	}
}

/*矩阵相关函数*/
