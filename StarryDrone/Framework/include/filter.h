/*
 * File      : filter.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __FILTER_H__
#define __FILTER_H__

#include <rtthread.h>
#include <rtdevice.h>

rt_err_t filter_init(void);
void accfilter_input(const float val[3]);
const float * accfilter_getCurrent(void);
void gyrfilter_input(const float val[3]);
const float * gyrfilter_current(void);
void magfilter_input(const float val[3]);
const float * magfilter_getCurrent(void);

#endif
