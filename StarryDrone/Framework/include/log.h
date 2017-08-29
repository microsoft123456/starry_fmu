/*
 * File      : log.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-22     zoujiachi   	the first version
 */
 
#ifndef __LOG_H__
#define __LOG_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef enum
{
	LOG_INTERFACE_SERIAL = 0,
	LOG_INTERFACE_USB
}LOG_INTERFACE_Typedef;

typedef struct
{
	void (*e)(char* tag, const char *fmt, ...);
	void (*w)(char* tag, const char *fmt, ...);
	void (*console)(const char *fmt, ...);
	void (*eachtime)(uint32_t *time_stamp, uint32_t time_ms, const char *fmt, ...);
}LOG_Typedef;

extern LOG_Typedef Log;

uint8_t log_init(LOG_INTERFACE_Typedef log_if);

#endif
