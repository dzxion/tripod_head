#ifndef _CAN_H_
#define _CAN_H_
#include "sys.h"
#define CANID  0x502
#define CAN_BAUDRATE_1M  	  1000	//1MBps
#define CAN_BAUDRATE_500K   500		//500kBps
#define CAN_BAUDRATE_250K   250		//250kBps
#define CAN_BAUDRATE_125K   125		//125kBps
#define CAN_BAUDRATE_100K   100		//100kBps 
	
void CanHardwareInit(u16 CAN_BAUDRATE);

uint8_t CanSend(u32 id, u8 *data, u8 size);
uint16_t can_serial_available(void);
uint8_t can_serial_read_char(void);
uint16_t can_serial_write(uint8_t *buffer, uint16_t length); //∑¢ÀÕ
uint16_t can_serial_read(uint8_t *buffer, uint16_t length);  //Ω” ’
void can_transmit(void);
void up_grade(uint8_t ctx);
	
extern u8 Fc_Data[9];
#endif


