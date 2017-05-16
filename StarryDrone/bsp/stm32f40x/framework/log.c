/*
 * File      : log.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-22     zoujiachi   	the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define LOG_BUFF_SIZE		128
static char log_buf[LOG_BUFF_SIZE];

LOG_Typedef Log;
static rt_device_t log_device;

extern rt_device_t _console_device;

//redefine fputc(),for printf() function to call
int fputc(int ch, FILE * file)
{
	if(log_device != NULL)
		rt_device_write(log_device, 0, (void*)&ch, 1);

	return ch; 
}

void log_output(char* content, uint32_t len)
{
	rt_device_write(log_device, 0, (void*)content, len);
}

void log_error(char* tag, const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(log_buf, fmt, args);
	va_end(args);
	
	log_output(log_buf, length);
}

void log_warning(char* tag, const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(log_buf, fmt, args);
	va_end(args);
	
	log_output(log_buf, length);
}

void log_eachtime(uint32_t *time_stamp, uint32_t time_ms, const char *fmt, ...)
{
	uint32_t now = time_nowMs();
	if(now - *time_stamp > time_ms){
		*time_stamp = now;
		
		va_list args;
		int length;
		
		va_start(args, fmt);
		length = vsprintf(log_buf, fmt, args);
		va_end(args);
		
		log_output(log_buf, length);
	}
}

uint8_t log_init(LOG_INTERFACE_Typedef log_if)
{	
	Log.e = log_error;
	Log.w = log_warning;
	Log.eachtime = log_eachtime;
	
	if(log_if == LOG_INTERFACE_SERIAL)
		log_device = rt_device_find("uart3");
	else 
		log_device = rt_device_find("usb");
	if(log_device)
		rt_device_open(log_device , RT_DEVICE_OFLAG_RDWR);
	
	return 0;
}

