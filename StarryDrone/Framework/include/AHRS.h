/*
 * File      : AHRS.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __AHRS_H__
#define __AHRS_H__

#include "quaternion.h"
#include "ap_math.h"

void AHRS_reset(quaternion * q, const float acc[3],const float mag[3]);
void AHRS_update(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT);

#endif
