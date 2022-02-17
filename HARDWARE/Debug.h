#ifndef _Debug_H_
#define _Debug_H_
#include "sys.h"

typedef struct 
{
	float data1;
	float data2;
	float data3;
	float data4;

	float data5;
	float data6;
	float data7;
	float data8;

	float data9;
	float data10;
	float data11;
	float data12;

	float data13;
	float data14;
	float data15;
	float data16;
}_debug_data;

extern _debug_data Debug_Data;

void Oscilloscope(void);

#endif



