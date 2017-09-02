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

#define PX4IO_THREAD_PRIORITY		5
#define ATTITUDE_THREAD_PRIORITY	6
#define POS_THREAD_PRIORITY			7
#define CONTROL_THREAD_PRIORITY		8
#define MAVLINK_THREAD_PRIORITY		9

extern const float PI;

typedef enum
{
	false = 0,
	true = 1,
}bool;

#endif
