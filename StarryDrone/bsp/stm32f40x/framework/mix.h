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

#include <rtthread.h>
#include <rtdevice.h>

void mix_init(void);
void mix_gyr(quaternion * attitude,const float gyr[3],float interval);
void mix_AccMag(quaternion * attitude,const float acc[3],const float mag[3]);
void mix_AccMag_steepestDescentMethod(quaternion * attitude,const float acc[3],const float mag[3]);
void mix_Acc_steepestDescentMethod(quaternion * attitude,const float acc[3]);
void mix_gyrAcc_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],float interval);
void mix_gyrAccMag_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],const float mag[3],float interval);
void mix_Acc_FullMix(quaternion * attitude , const float acc[3]);
void mix_Mag_FullMix(quaternion * attitude , const float mag[3]);

#endif
