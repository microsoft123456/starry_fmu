/*
* File      : mavlink_customer.h
*
*
* Change Logs:
* Date			Author			Notes
* 2016-06-23	zoujiachi		the first version
*/

#include <rtthread.h>
#include <rtdevice.h>

rt_err_t device_mavlink_init(void);
void mavlink_loop(void *parameter);
