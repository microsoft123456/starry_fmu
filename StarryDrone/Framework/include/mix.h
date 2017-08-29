/*
 * File      : mix.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __MIX_H__
#define __MIX_H__

//#include <rtthread.h>
//#include <rtdevice.h>
#include "quaternion.h"
#include "ap_math.h"

void mix_init(void);
void mix_gyrAcc_crossMethod(quaternion * q,const float gyr[3],const float acc[3],float dT);
void mix_gyrAccMag_crossMethod(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT);

#endif
