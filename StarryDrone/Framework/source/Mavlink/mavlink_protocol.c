/*
* File      : mavlink.c
*
*
* Change Logs:
* Date			Author			Notes
* 2016-06-23	zoujiachi		the first version
*/

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include "log.h"
#include "delay.h"
#include "quaternion.h"
#include "position.h"
#include "attitude.h"
#include "mavlink_protocol.h"

#define EVENT_MAV_1HZ_UPDATE		(1<<0)
#define EVENT_MAV_3HZ_UPDATE		(1<<1)

static rt_device_t usb_device = NULL;
uint8_t mav_tx_buff[1024];
mavlink_system_t mavlink_system;
/* disable mavlink sending */
uint8_t mav_disenable = 0;

static char *TAG = "Mavlink";

static struct rt_timer timer_1HZ;
static struct rt_timer timer_3HZ;

struct rt_event event_mavlink;

extern rt_device_t _console_device;

uint8_t mavlink_msg_transfer(uint8_t chan, uint8_t* msg_buff, uint16_t len)
{
	if(usb_device){
		rt_device_write(usb_device, 0, (void*)msg_buff, len);
	}else{
		rt_uint16_t old_flag = _console_device->open_flag;

		_console_device->open_flag |= RT_DEVICE_FLAG_STREAM;
		rt_device_write(_console_device, 0, msg_buff, len);
		_console_device->open_flag = old_flag;
	}
	
	return 0;
}

rt_err_t device_mavlink_init(void)
{
	mavlink_system.sysid = 20;                   
	mavlink_system.compid = MAV_COMP_ID_IMU;     

	return 0;
}

uint8_t mavlink_send_msg_heartbeat(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0xFE, 0, system_status);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

uint8_t mavlink_send_msg_attitude_quaternion(uint8_t system_status, quaternion attitude)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_attitude_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       0, attitude.w, attitude.x, attitude.y, attitude.z,0,0,0);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

uint8_t mavlink_send_msg_global_position(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	Position_Info pos_info;
	
	if(mav_disenable)
		return 0;
	
	pos_info = get_pos_info();
	
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       0, pos_info.lat, pos_info.lon, pos_info.alt, pos_info.relative_alt, 
								pos_info.vx, pos_info.vy, pos_info.vz, UINT16_MAX);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

uint8_t mavlink_send_msg_rc_channels_raw(uint32_t channel[8])
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       time_nowMs(), 1, (uint16_t)1000*channel[0], (uint16_t)1000*channel[1], 
								(uint16_t)1000*channel[2], (uint16_t)1000*channel[3], (uint16_t)1000*channel[4], (uint16_t)1000*channel[5], 
								(uint16_t)1000*channel[6], (uint16_t)1000*channel[7], 70);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

static void timer_mavlink_1HZ_update(void* parameter)
{
	rt_event_send(&event_mavlink, EVENT_MAV_1HZ_UPDATE);
}

static void timer_mavlink_3HZ_update(void* parameter)
{
	rt_event_send(&event_mavlink, EVENT_MAV_3HZ_UPDATE);
}

void mavlink_loop(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_MAV_1HZ_UPDATE | EVENT_MAV_3HZ_UPDATE;
	
	Log.w(TAG, "mavlink_loop\r\n");
	/* create event */
	res = rt_event_init(&event_mavlink, "mavlink_event", RT_IPC_FLAG_FIFO);
	
	usb_device = rt_device_find("usb");
	if(usb_device == NULL)
		Log.e(TAG, "err not find usb device\n");
	else
		rt_device_open(usb_device , RT_DEVICE_OFLAG_RDWR);
	
	/* register timer event */
	rt_timer_init(&timer_1HZ, "timer_1HZ",
					timer_mavlink_1HZ_update,
					RT_NULL,
					1000,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_1HZ);
	
	rt_timer_init(&timer_3HZ, "timer_3HZ",
					timer_mavlink_3HZ_update,
					RT_NULL,
					100,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_3HZ);
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_mavlink, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK)
		{
			if(recv_set & EVENT_MAV_1HZ_UPDATE)
			{
				mavlink_send_msg_heartbeat(MAV_STATE_STANDBY);	
				mavlink_send_msg_global_position(MAV_STATE_STANDBY);
			}
			
			if(recv_set & EVENT_MAV_3HZ_UPDATE)
			{
				mavlink_send_msg_attitude_quaternion(MAV_STATE_STANDBY, attitude_getAttitude());
			}
		}
		else
		{
			//some err happen
			Log.e(TAG, "mavlink loop, err:%d\r\n" , res);
		}
	}
}

