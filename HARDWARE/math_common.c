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

void get_Rotation_vec( vector3* vec, Quaternion q )
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

void Quat2Euler( float* roll, float* pitch, float* yaw, Quaternion q )
{
	*roll = atan2f( 2.0f*(q.qw*q.qx+q.qy*q.qz) , 1.0f-2.0f*(q.qx*q.qx+q.qy*q.qy) );
	*pitch = asinf( 2.0f*(q.qw*q.qy-q.qx*q.qz) );
	*yaw = atan2f( 2.0f*(q.qw*q.qz+q.qx*q.qy) , 1.0f-2.0f*(q.qy*q.qy+q.qz*q.qz) );
}
/*旋转相关函数*/
