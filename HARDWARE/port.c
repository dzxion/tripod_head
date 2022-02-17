#include "port.h"
#include "can.h"

static port_t* port_list[PORT_MAX];

uint8_t Port_Register(uint8_t port_type, port_t *port)
{
    if(port_type < PORT_MAX) {
        assert_param(port != 0);
        port_list[port_type] = port;
        return 1;
    } else {
        return 0;
    }
}

port_t *Port_Get(uint8_t port_type)
{
    if(port_list[port_type] != NULL) {
        return port_list[port_type];
    } else {
        return NULL;
    }
}

