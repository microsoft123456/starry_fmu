/*
 * File      : pid.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-01     zoujiachi    first version.
 */

#ifndef _PID_H__
#define _PID_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef struct
{
	float inner_P , inner_I , inner_D;
	float outer_P , outer_I , outer_D;
}PID_Param;

#endif
