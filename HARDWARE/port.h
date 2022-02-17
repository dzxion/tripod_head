#ifndef _PORT_H_H_
#define _PORT_H_H_

#include "stm32f30x.h"
#include "string.h"


enum {
    PORT_USART,
    PORT_CAN,
	PORT_MAX,
};

typedef struct
{
    uint8_t *data;
    volatile uint16_t read_index, write_index;
    uint16_t size;
} Can_fifo_buffer_t;

typedef struct {
    uint16_t (*available)(void);
    uint8_t  (*read_one)(void);
    uint16_t (*read)(uint8_t *data, uint16_t len);
    uint16_t (*write)(uint8_t *data, uint16_t len);
} port_t;

uint8_t Port_Register(uint8_t port_type, port_t *port);
port_t *Port_Get(uint8_t port_type);

#endif
