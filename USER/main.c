#include "app.h"
#pragma pack(1)
typedef struct
{
	u8 rev1[2];
  u8 flag;
	float xxa1;
	float xxa2;
}TEST_t; 
#pragma pack()

union rdata{
    u32 frt;
	  float xa1;
};
union rdata rdatay;

#pragma pack(1)
typedef struct
{
	u8 rev1[2];
  u8 flag;
	float xxa1;
	float xxa2;
}TEST2_t; 
#pragma pack()

TEST_t TEST;
TEST2_t TEST2;

float etet1 = 0.2f;
float etet2 = 0.0f;

u8 test_buff[11];
u8 test_len1 = 0;
u8 test_len2 = 0;



int main(void)
{
	delay_init(72);
	
	
//	Core_Check();// 必须执行初始化
	Gimbal_Init();
//	TEST.flag = 1;
//	TEST.rev1[0] = 2;
//	TEST.rev1[1] = 3;
//	TEST.xxa1 = 0.1f;
//	TEST.xxa2 = 0.2f;
//	delay_ms(10);
//	test_len1 = sizeof(TEST);
//	memcpy(&test_buff,&TEST,sizeof(TEST));	
	test_buff[0] = 0x02;
	test_buff[1] = 0x03;
	test_buff[2] = 0x01;
	test_buff[3] = 0xCD;
	test_buff[4] = 0xCC;
	test_buff[5] = 0xCC;
	test_buff[6] = 0x3D;
	test_buff[7] = 0xCD;
	test_buff[8] = 0xCC;
	test_buff[9] = 0x4C;
	test_buff[10]= 0x3E;
	
	test_len2 = sizeof(TEST2);
//	rdatay.xa1 =etet1;
	delay_ms(10);
//	memcpy(&rdatay.frt,&test_buff[7],4);	
	memcpy(&TEST2,test_buff,sizeof(TEST2));
	init_MS_Attitude();
	
  while (1)
  {		
//		R_LED_Status(3);
//			G_LED_Status(4);
//		delay_us(500);
    Gimbal_Control();
  }
}

