#ifndef __math_common_H
#define __math_common_H

#include "sys.h" 

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

typedef struct
{
	float x;
	float y;
	float z;
}vector3;

typedef struct
{
	float qw;
	float qx;
	float qy;
	float qz;
}Quaternion; 

typedef struct
{  
  float a11,a12,a13;
  float a21,a22,a23;
  float a31,a32,a33;	
}Matrix33;

void normalize( Quaternion* q );
void Euler2Quat( Quaternion* q, float roll, float pitch, float yaw );
void Quat2Euler( vector3* euler, Quaternion q );
void Quat_Prod( Quaternion* q, Quaternion a, Quaternion b );
void integral( Quaternion* q, vector3 delta_angle );
void conjugate( Quaternion* q );
void Quat2Axis( vector3* vec, Quaternion q );
void Axis2Quat( Quaternion* q, vector3 vec );
void Quat2Mat( Matrix33* mat, Quaternion q );
void rotate( vector3* vec_out, vector3 vec_in, Matrix33 mat);
void Mat66mul(float*C,float A[36],float B[36]);
void Mat_T66(float*C,float A[36]);
void Euler2Quat_G( Quaternion* q, float roll, float pitch, float yaw );
void Quat2Euler_G( vector3* euler, Quaternion q );

#endif
