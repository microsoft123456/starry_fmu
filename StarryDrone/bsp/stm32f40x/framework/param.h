/*
 * File      : param.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __PARAM_H__
#define __PARAM_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef struct
{
	/* firmware version */
	uint16_t version;
	
	/* sensor compensate param */
	float acc_offset[3];
	float acc_gain[3];
	float mag_offset[3];
	float mag_gain[3];
	float gyr_offset[3];
	float gyr_gain[3];
	
	quaternion initial_attitude;
	
	/* control pid param */
	struct
	{
		float inner_P , inner_I , inner_D;
		float outer_P , outer_I , outer_D;
	}ctl_pid[3];
	
	/* halt voltage */
	float halt_vol;
	
	/* halt incline cos*/
	float halt_incline_cos;
}PARAM_Def;

rt_err_t param_init(void);
const PARAM_Def * get_param(void);

#endif
