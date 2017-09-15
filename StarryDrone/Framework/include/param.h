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

#include <stm32f4xx.h>
#include "quaternion.h"

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
	
	/* attitude control pid param */
	float att_angle_p[3];
	float att_rate_p[3];
	float att_rate_i[3];
	float att_rate_d[3];
	
	/* halt voltage */
	float halt_vol;
	
	/* halt incline cos*/
	float halt_incline_cos;
}PARAM_Def;

uint8_t param_init(void);
const PARAM_Def * get_param(void);

#endif
