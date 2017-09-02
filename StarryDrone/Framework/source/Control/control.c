/*
 * File      : control.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-25     zoujiachi    first version.
 */
 
//#include <rthw.h>
#include <rtdevice.h>
#include <math.h>
#include "rc.h"
#include "pwm.h"
#include "control.h"
#include "quaternion.h"
#include "attitude.h"
#include "motor.h"
#include "global.h"
#include "log.h"
#include "filter.h"
#include "pid.h"

#define EVENT_ATT_CTRL			(1<<0)

#define ATT_CTRL_INTERVAL		10
/* the control angle for rc(degree) */
#define ANGLE_CONTROL_SCALE		(30.0f)
/* the maximal yaw rotation speed is 30 deg/s */
#define YAW_CONTROL_SPEED		(30.0f)

static rt_device_t motor_device_t;
static struct rt_timer timer_att;
static struct rt_event event_ctrl;
static float throttle_base[MAX_THROTTLE_NUM];
static float yaw_target;
/* 0:lock	1:unlock*/
static float _vehicle_status = 0;

static char* TAG = "Control";

static void timer_att_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_ctrl, EVENT_ATT_CTRL);
}

//////////////////////////////////////////////////////////////////////

void ctrl_init(void)
{
	quaternion cur_att = attitude_getAttitude();
	yaw_target = quaternion_getEuler(cur_att, 2);
}

void set_throttle_base(float* throttle, uint8_t throttle_num)
{
	if(throttle_num > MAX_THROTTLE_NUM)
		return ;
	
	for(uint8_t i = 0 ; i<throttle_num ; i++)
	{
		throttle_base[i] = throttle[i];
	}
	
	//for motor test
	rt_device_write(motor_device_t, MOTOR_CH_ALL, throttle_base, 4);
}

quaternion calc_target_att(int ctrl_mode)
{
	quaternion qt;	// target quaternion
	static float ctrl_dT = (float)ATT_CTRL_INTERVAL/1000.0f;
	if(ctrl_mode == 1){	/* attitude control */
		float toRad = PI/180.0f;
		float euler[3];
		
		euler[0] = (rc_get_chanval(CHAN_ROLL)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
        euler[1] = (rc_get_chanval(CHAN_PITCH)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
		yaw_target += (rc_get_chanval(CHAN_YAW)-0.5f)*2.0f*ctrl_dT*toRad*YAW_CONTROL_SPEED;
		if(yaw_target >= 360.0f){
			yaw_target -= 360.0f;
		}
		euler[2] = yaw_target;
		
		quaternion_fromEuler(euler, &qt);
	}
}

void ctrl_unlock_vehicle(void)
{
	int on_off = 1;
	rt_device_control(motor_device_t, PWM_CMD_SWITCH, (void*)&on_off);
	_vehicle_status = 1;
}

void ctrl_lock_vehicle(void)
{
	int on_off = 0;
	rt_device_control(motor_device_t, PWM_CMD_SWITCH, (void*)&on_off);
	_vehicle_status = 0;
}

void control_loop(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_ATT_CTRL;
	motor_device_t = rt_device_find("motor");
	
	if(motor_device_t == RT_NULL)
	{
		Log.e(TAG, "err, can not find motor device\n");
		return ;
	}
	
	rt_device_open(motor_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* create event */
	res = rt_event_init(&event_ctrl, "att_event", RT_IPC_FLAG_FIFO);
	
	/* register timer event */
	rt_timer_init(&timer_att, "timer_att",
					timer_att_update,
					RT_NULL,
					ATT_CTRL_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_att);
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_ctrl, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK)
		{
			if(recv_set & EVENT_ATT_CTRL)
			{
				float att_err[3];
				float out[3];
				float gyr[3];
				const float *gyr_t = gyrfilter_current();
				gyr[0] = gyr_t[0];
				gyr[1] = gyr_t[1];
				gyr[2] = gyr_t[2];
				pid_calculate(att_err, out, gyr);
			}
		}else
		{
			//some err occur
			Log.e(TAG, "control loop, err:%d\r\n" , res);
		}
	}
}
