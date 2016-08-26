
#ifndef __COM_SERIAL_H__
#define __COM_SERIAL_H__

#include <rtthread.h>
#include <rtdevice.h>

#define COM_SERIAL_CONTENT_SIZE_MAX (32-1-1-2)

int32_t com_serial_init(void);
void    com_serial_checkEvent(void);
int32_t com_serial_transmitBegin(uint8_t packetType);
int32_t com_serial_transmitContent(const void * part,int32_t length);
int32_t com_serial_transmitEnd(void);

#endif