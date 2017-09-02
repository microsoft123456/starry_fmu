/*
 * File      : remote_controller.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-29     zoujiachi   	the first version
 */

#ifndef __REMOTE_CONTROLLER_H__
#define __REMOTE_CONTROLLER_H__

#include <stdint.h>

#define CHAN_NUM			6

//#define CHAN_THROTTLE		3
//#define CHAN_YAW			4
//#define CHAN_ROLL			1
//#define CHAN_PITCH			2

typedef enum
{
	CHAN_ROLL = 0,	//channel 1
	CHAN_PITCH,
	CHAN_THROTTLE,
	CHAN_YAW,
	CHAN_5,
	CHAN_6
}RC_CHANEL;

inline float rc_raw2chanval(uint32_t raw);
float rc_get_chanval(RC_CHANEL rc_chan);
void rc_handle_ppm_signal(float* chan_val);

#endif
