/*
 * File      : gps_sensor.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-06-30     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

static rt_device_t serial_device;

rt_err_t rt_gps_init(char* serial_device_name)
{	
	rt_err_t res = RT_EOK;
	
	serial_device = rt_device_find(serial_device_name);
	
	if(serial_device == RT_NULL)
    {
        rt_kprintf("serial device %s not found!\r\n", serial_device);
        return RT_EEMPTY;
    }
	
	rt_device_open(serial_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
//	u8 ch , byte;
//	while(1)
//	{
//		byte = rt_device_read(serial_device , 0 , &ch , 1);
//		if(byte)
//		{
//			rt_kprintf("%c" , ch);
//		}
//	}
	
	return res;
}
