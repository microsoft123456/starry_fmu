/*
 * File      : attitude.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <rtthread.h>
#include "quaternion.h"

rt_err_t attitude_init(void);
quaternion attitude_getAttitude(void);
void attitude_inputAcc(const float acc[3]);
void attitude_inputGyr(const float gyr[3]);
void attitude_inputMag(const float mag[3]);
void attitude_loop(void *parameter);

#endif
