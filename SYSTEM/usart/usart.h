#ifndef __USART_H
#define __USART_H
#include "stm32f30x.h"

#define BUFFER_SIZE   (128)

extern uint8_t Tx3_buffer[BUFFER_SIZE];
extern uint8_t Rx3_buffer[BUFFER_SIZE];

typedef struct
{
    uint8_t *data;
    uint16_t read_index, write_index;
    uint16_t size;
}fifo_buffer_t;


extern u8 U2_IDLE_Flag;
extern u8 U3_IDLE_Flag;

void USART1_init(u32 bound);
void USART2_init(u32 bound);
void USART3_init(u32 bound);
u16 Get_CamData(void);

uint32_t UART1_SendDataDMA(uint8_t *data, uint16_t len);
uint32_t UART2_SendDataDMA(uint8_t *data, uint16_t len);
uint32_t UART3_SendDataDMA(uint8_t *data, uint16_t len);

uint16_t Serial_available(void);
uint8_t Serial_read_char(void);
uint16_t Serial_read(uint8_t *buffer, uint16_t length);
	
#endif
