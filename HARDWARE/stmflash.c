#include "stmflash.h"

u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}

void STMFLASH_Write(u32 WriteAddr ,u16 *Data,u16 NumToWrite)
{
	u16 i;
  FLASH_Unlock();	//����
	FLASH_ErasePage(WriteAddr); //0X08020000,128*1024=131072=0x20000
	                             //ҳ��ַ 2048 * ҳ ת���� 16����
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,Data[i]);
		WriteAddr+=2;//��ַ����2.
	}
	  FLASH_Lock();//����	
}

void STMFLASH_Read(u32 WriteAddr ,u16 *Data,u16 NumToWrite)
{
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		Data[i] = STMFLASH_ReadHalfWord(WriteAddr);
		WriteAddr+=2;//��ַ����2.
	}
}
