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
uint8_t mavlink_send_msg_rc_channels_raw(uint32_t channel[8]);
