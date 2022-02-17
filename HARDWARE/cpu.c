#include "app.h"

void cpu_reset(void)
{
 NVIC_SystemReset();
}

void mcu_get_uid(void *uid)
{ 
	memcpy((uint8_t*)uid,(uint8_t*)"\\x00\x00\x00\x05\x00\x00\x00\x05\x10\x00\x00\x18",12);
}
