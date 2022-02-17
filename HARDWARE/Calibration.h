#ifndef __Calibration_H
#define __Calibration_H
#include "sys.h" 

extern u16 Gimbal_CaliData[320];

void Temp_pid(u8 SetTemp); //Electrical angle
void Gyro_Cali(void);


#endif
