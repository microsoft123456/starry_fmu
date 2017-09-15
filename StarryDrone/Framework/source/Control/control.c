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
#include "px4io_manager.h"

#define EVENT_ATT_CTRL			(1<<0)

#define ATT_CTRL_INTERVAL		10
/* the control angle for rc(degree) */
#define ANGLE_CONTROL_SCALE		(20.0f)
/* the maximal yaw rotation speed 20 deg/s */
#define YAW_CONTROL_SPEED		(20.0f)

static rt_device_t motor_device_t;
static struct rt_timer timer_att;
static struct rt_event event_ctrl;
static float _baseThrottle = 0;
static float yaw_target;
/* 0:lock	1:unlock*/
static float _vehicle_status = 0;
static float _throttle_out[MOTOR_NUM];

static char* TAG = "Control";

static void timer_att_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_ctrl, EVENT_ATT_CTRL);
}

void ctrl_constrain_throttle(float* throttle, uint8_t throttle_num)
{
	for(int i = 0 ; i < throttle_num ; i++){
		if(throttle[i] < 0.0f)
			throttle[i] = 0.0f;
		if(throttle[i] > 0.8f)
			throttle[i] = 0.8f;
	}
}

//////////////////////////////////////////////////////////////////////

void ctrl_set_throttle(float* throttle, uint8_t throttle_num)
{
	if(throttle_num > MAX_THROTTLE_NUM){
		Log.w(TAG, "throttle num:%d is larger than max throttle num:%d\n", throttle_num, MAX_THROTTLE_NUM);
		return ;
	}

	rt_device_write(motor_device_t, MOTOR_CH_ALL, throttle, throttle_num);
}

quaternion calc_target_quaternion(int ctrl_mode, float dT)
{
	quaternion qt;	// target quaternion

	if(ctrl_mode == 1){	/* attitude control */
		float toRad = PI/180.0f;
		Euler e;
		
		e.roll = (rc_get_chanval(CHAN_ROLL)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
        e.pitch = -(rc_get_chanval(CHAN_PITCH)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
		if(rc_get_chanval(CHAN_YAW)>0.55 || rc_get_chanval(CHAN_YAW)<0.45){
			yaw_target += (rc_get_chanval(CHAN_YAW)-0.5f)*2.0f*dT*toRad*YAW_CONTROL_SPEED;
			if(yaw_target >= PI){
				yaw_target -= 2*PI;
			}
			if(yaw_target < -PI){
				yaw_target += 2*PI;
			}
		}
		e.yaw = yaw_target;
		
		quaternion_fromEuler(e, &qt);
	}
	
	return qt;
}

Euler calc_target_euler(int ctrl_mode, float dT)
{
	Euler et;	// target quaternion

	if(ctrl_mode == 1){	/* attitude control */
		float toRad = PI/180.0f;
		
		et.roll = (rc_get_chanval(CHAN_ROLL)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
        et.pitch = -(rc_get_chanval(CHAN_PITCH)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
		if(rc_get_chanval(CHAN_YAW)>0.55 || rc_get_chanval(CHAN_YAW)<0.45){
			yaw_target += (rc_get_chanval(CHAN_YAW)-0.5f)*2.0f*dT*toRad*YAW_CONTROL_SPEED;
			if(yaw_target >= PI){
				yaw_target -= 2*PI;
			}
			if(yaw_target < -PI){
				yaw_target += 2*PI;
			}
		}
		et.yaw = yaw_target;
	}
	
	return et;
}

void ctrl_unlock_vehicle(void)
{
	int on_off = 1;
	
	quaternion cur_att = attitude_getAttitude();
	yaw_target = quaternion_getEuler(cur_att, 2);
	
	rt_device_control(motor_device_t, PWM_CMD_SWITCH, (void*)&on_off);
	_vehicle_status = 1;
	
	pid_init();
}

void ctrl_lock_vehicle(void)
{
	int on_off = 0;
	
	for(int i = 0 ; i < MOTOR_NUM ; i++){
		_throttle_out[i] = 0.0f;
	}
	ctrl_set_throttle(_throttle_out, MOTOR_NUM);
	rt_device_control(motor_device_t, PWM_CMD_SWITCH, (void*)&on_off);
	_vehicle_status = 0;
}

void control_init(void)
{
	motor_device_t = rt_device_find("motor");
	if(motor_device_t == RT_NULL){
		Log.e(TAG, "can't find motor device\n");
		return;
	}
	rt_device_open(motor_device_t , RT_DEVICE_OFLAG_RDWR);
}

void control_attitude(void)
{
	float att_err[3];
	float out[3];
	static float gyr[3] = {0, 0, 0};
	const float *gyr_t = gyrfilter_current();
	float alpha = 0.4;

	/* unlock status */
	if(_vehicle_status){
		/* get throttle value */
		_baseThrottle = rc_get_chanval(CHAN_THROTTLE);
		
		/* only control attitude when throttle is bigger then a specific threshold */
		if(_baseThrottle > 0.05){	
			gyr[0] = gyr_t[0];
			gyr[1] = gyr_t[1];
			gyr[2] = gyr_t[2];

			quaternion qc = attitude_getAttitude();
			
			/* current attitude */
			Euler ec;
			quaternion_toEuler(qc, &ec);
			
			/* tilt protection */
			if(abs(Rad2Deg(ec.roll))>45 || abs(Rad2Deg(ec.pitch))>45){
				rc_enter_status(RC_LOCK_STATUS);
				return ;
			}
			
			/* calculate target attitude according to rc value */
			Euler et = calc_target_euler(1, 1.0f/(float)ppm_send_freq);

			att_err[0] = Rad2Deg((et.roll - ec.roll));
			att_err[1] = Rad2Deg((et.pitch - ec.pitch));
			att_err[2] = Rad2Deg((et.yaw - ec.yaw));
			/* transfer yaw range to 0~360 deg */
			if(att_err[2] > 180.0f){
				att_err[2] -= 360.0f;
			}
			if(att_err[2] < -180.0f){
				att_err[2] += 360.0f;
			}
			
			/* pid controller */
			pid_calculate(att_err, out, gyr, (float)ATT_CTRL_INTERVAL/1000.0f);

			/* motor output matrix, mix pid output with base throttle */
			/* -1	 1	-1 */
			/* -1	-1	 1 */
			/*  1	-1	-1 */
			/*  1	 1	 1 */
			_throttle_out[0] = _baseThrottle - out[0] + out[1] - out[2];
			_throttle_out[1] = _baseThrottle - out[0] - out[1] + out[2];
			_throttle_out[2] = _baseThrottle + out[0] - out[1] - out[2];
			_throttle_out[3] = _baseThrottle + out[0] + out[1] + out[2];
			
//			static uint32_t time = 0;
//			Log.eachtime(&time, 10, "%.2f %.2f %.2f\n", Rad2Deg(ec.pitch),Rad2Deg(et.pitch), gyr[1]);
		}else{
			_throttle_out[0] = _baseThrottle;
			_throttle_out[1] = _baseThrottle;
			_throttle_out[2] = _baseThrottle;
			_throttle_out[3] = _baseThrottle;
		}
		
		ctrl_constrain_throttle(_throttle_out, MOTOR_NUM);
		ctrl_set_throttle(_throttle_out, MOTOR_NUM);
		
		
		//Log.eachtime(&time, 300, "out:%.2f %.2f %.2f err:%.2f %.2f %.2f\n", out[0],out[1],out[2],att_err[0],att_err[1],att_err[2]);
	}
}
