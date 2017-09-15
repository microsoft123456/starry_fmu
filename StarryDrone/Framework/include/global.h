/*
 * File      : global.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-10-02     zoujiachi   	the first version
 */
 
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdio.h>
#include <stdlib.h>

//#define PX4IO_THREAD_PRIORITY		5
//#define ATTITUDE_THREAD_PRIORITY	6
//#define POS_THREAD_PRIORITY			7
//#define CONTROL_THREAD_PRIORITY		8
//#define MAVLINK_THREAD_PRIORITY		9

#define PX4IO_THREAD_PRIORITY		8
#define ATTITUDE_THREAD_PRIORITY	5
#define POS_THREAD_PRIORITY			6
#define CONTROL_THREAD_PRIORITY		7
#define MAVLINK_THREAD_PRIORITY		9

//#define Rad2Deg(x)			(x*57.2957795f)
//#define Deg2Rad(x)			(x*0.0174533f)
#define Rad2Deg(x)			((x)*180.0f/PI)
#define Deg2Rad(x)			((x)*PI/180.0f)

extern const float PI;

typedef enum
{
	false = 0,
	true = 1,
}bool;

#endif
