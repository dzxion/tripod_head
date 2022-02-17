#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  

u16 STMFLASH_ReadHalfWord(u32 faddr);		  //¶Á³ö°ë×Ö  
void STMFLASH_Write(u32 WriteAddr ,u16 *Data,u16 NumToWrite);
void STMFLASH_Read(u32 WriteAddr ,u16 *Data,u16 NumToWrite);

#endif

















