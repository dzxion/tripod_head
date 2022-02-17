#ifndef __CPU_H__
#define __CPU_H__
#include "stdint.h"

#define UUID_ADDRESS (0x1FFFF7E8)

void mcu_get_uid(void *uid);

void cpu_reset(void);

#endif  /* __CPU_H__ */
