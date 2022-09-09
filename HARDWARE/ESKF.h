#include "math_common.h"
#include "sys.h" 

void MS_Attitude_ESKF_Bias_FC(Quaternion* q,vector3* bias,vector3 gyro,vector3 angle_vehicle,vector3 encoder,u8* step);
