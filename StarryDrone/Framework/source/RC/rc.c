/*
 * File      : remote_controller.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-29     zoujiachi   	the first version
 */

#include <string.h>
#include "global.h"
#include "log.h"
#include "motor.h"
#include "rc.h"
#include "delay.h"
#include "control.h"

/* If more than RC_LOST_SIGNAL_TIME ms we don't receive ppm signal, then we think rc is disconnected */
#define RC_LOST_SIGNAL_TIME		300

typedef enum
{
	RC_LOCK_STATUS = 0,
	RC_LOCK_WAIT_STATUS,
	RC_UNLOCK_STATUS,
	RC_UNLOCK_WAIT_STATUS,
}RC_STATUS;

typedef enum
{
	RC_LOCK_SIGNAL = 0,
	RC_UNLOCK_SIGNAL,
	RC_RETURN_CENTRAL_SIGNAL,
	RC_NORMAL_SIGNAL,
}RC_SIGNAL;

static char* TAG = "RC";
static float _chan_val[CHAN_NUM];
static bool _rc_connect = false;
static uint32_t _time_last_receive = 0;
RC_STATUS _rc_status = RC_LOCK_STATUS;

inline float rc_raw2chanval(uint32_t raw)
{
	uint32_t raw_temp = raw;
	
	if(raw_temp > 2000)
		raw_temp = 2000;
	if(raw_temp < 1000)
		raw_temp = 1000;
	
	return (float)(raw_temp-1000)/1000;
}

float rc_get_chanval(RC_CHANEL rc_chan)
{
	if((int)rc_chan >= CHAN_NUM)
		return -1;
	
	return _chan_val[rc_chan];
}

bool rc_get_connect_status(void)
{
	if(time_nowMs() - _time_last_receive > RC_LOST_SIGNAL_TIME){
		_rc_connect = false;
	}else{
		_rc_connect = true;
	}
	
	return _rc_connect;
}

RC_SIGNAL rc_get_ppm_signal(void)
{
	if(_chan_val[CHAN_THROTTLE]<0.1 && _chan_val[CHAN_PITCH]<0.1){
		return RC_LOCK_SIGNAL;
	}else if(_chan_val[CHAN_THROTTLE]<0.1 && _chan_val[CHAN_PITCH]>0.9){
		return RC_UNLOCK_SIGNAL;
	}else if(_chan_val[CHAN_THROTTLE]<0.1 && _chan_val[CHAN_ROLL]>0.4 && _chan_val[CHAN_ROLL]<0.6
			&& _chan_val[CHAN_PITCH]>0.4 && _chan_val[CHAN_PITCH]<0.6){
		return RC_RETURN_CENTRAL_SIGNAL;
	}else{
		return RC_NORMAL_SIGNAL;
	}
}

int handle_rc_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "status") == 0){
			Log.console("rc connected: %s\n", rc_get_connect_status() ? "true" : "false");
			for(int i = 0 ; i < CHAN_NUM ; i++){
				Log.console("ch%d:%.2f ", i+1, _chan_val[i]);
			}
			Log.console("\n");
		}
	}
}

// chan_val: 0~1.0
void rc_handle_ppm_signal(float* chan_val)
{
	for(int i = 0 ; i < CHAN_NUM ; i++){
		_chan_val[i] = chan_val[i];
	}
	
	_time_last_receive = time_nowMs();
	
	switch((int)_rc_status)
	{
		case RC_LOCK_STATUS:
		{
			if(rc_get_ppm_signal() == RC_UNLOCK_SIGNAL){
				_rc_status = RC_UNLOCK_WAIT_STATUS;
				Log.console("enter RC_UNLOCK_WAIT_STATUS\n");
			}
		}break;
		case RC_LOCK_WAIT_STATUS:
		{
			if(rc_get_ppm_signal() == RC_RETURN_CENTRAL_SIGNAL){
				_rc_status = RC_LOCK_STATUS;
				ctrl_lock_vehicle();
				Log.console("enter RC_LOCK_STATUS\n");
			}
		}break;
		case RC_UNLOCK_STATUS:
		{
			if(rc_get_ppm_signal() == RC_LOCK_SIGNAL){
				_rc_status = RC_LOCK_WAIT_STATUS;
				Log.console("enter RC_LOCK_WAIT_STATUS\n");
			}
		}break;
		case RC_UNLOCK_WAIT_STATUS:
		{
			if(rc_get_ppm_signal() == RC_RETURN_CENTRAL_SIGNAL){
				_rc_status = RC_UNLOCK_STATUS;
				ctrl_unlock_vehicle();
				Log.console("enter RC_UNLOCK_STATUS\n");
			}
		}break;
		default:
		{
			Log.e(TAG, "rc unknown status:%d\n", _rc_status);
		}break;
	}

//	static uint32_t time_stamp = 0;
//	float euler[3];
//	quaternion_toEuler(qt, euler);
//	euler[0] *= 180.0/PI;
//	euler[1] *= 180.0/PI;
//	euler[2] *= 180.0/PI;
//	Log.eachtime(&time_stamp, 200, "roll:%.3f	pitch:%.3f	yaw:%.3f\n", euler[0], euler[1], euler[2]);
//	mavlink_send_msg_attitude_quaternion(MAV_STATE_STANDBY, qt);
	
	// for test
	float throttle[4] = {chan_val[CHAN_THROTTLE],chan_val[CHAN_THROTTLE],chan_val[CHAN_THROTTLE],chan_val[CHAN_THROTTLE]};
	ctrl_set_throttle(throttle, 4);
}