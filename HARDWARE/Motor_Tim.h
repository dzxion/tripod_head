#ifndef _Motor_Tim_H
#define _Motor_Tim_H
#include "sys.h"

#define PWM_FRQ		(20000)	
#define TS  		(unsigned short)(72*1000*1000/PWM_FRQ/2)

void SVPWM1_Init(void);
void SVPWM8_Init(void);
	
#endif


