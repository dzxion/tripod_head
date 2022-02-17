#ifndef _FOC_H_
#define _FOC_H_

#include "stm32f30x.h"
#include "math.h"

#define OppositePR 4 //俯仰和横滚电机型号: 4=小电机 7=大电机
        #define MotorPR_Radian (float)(OppositePR/57.295779513082320876798154814105f)

        #define OppositeY 7 // YAW电机
        #define MotorY_Radian  (float)(OppositeY/57.295779513082320876798154814105f)

STM32F303_RAMFUNC void MotorOut_PR(float Theta,float Vq,float Vd);
STM32F303_RAMFUNC void MotorOut_Y(float Theta,float Vq,float Vd);

#endif
