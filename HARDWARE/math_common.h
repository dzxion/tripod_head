#ifndef __math_common_H
#define __math_common_H

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

void normalize( Quaternion* q );
void Euler2Quat( Quaternion* q, float roll, float pitch, float yaw );
void Quat2Euler( float* roll, float* pitch, float* yaw, Quaternion q );
void Quat_Prod( Quaternion* q, Quaternion a, Quaternion b );
void integral( Quaternion* q, vector3 delta_angle );
void conjugate( Quaternion* q );
void get_Rotation_vec( vector3* vec, Quaternion q );

#endif
