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
//#include <rtdevice.h>

typedef struct{
    float	_cutoff_freq; 
    float	_a1;
    float	_a2;
    float	_b0;
    float	_b1;
    float	_b2;
    float	_delay_element_1;        // buffered sample -1
    float	_delay_element_2;        // buffered sample -2
}Butter2;

typedef struct
{
	int fir_length;	/* fir_length = fir_order+1 */
	int fir_index;
	float *fir_coeff;
	float *fir_buffer;
}FIR;

rt_err_t filter_init(void);
void accfilter_input(const float val[3]);
const float * accfilter_getCurrent(void);
void gyrfilter_input(const float val[3]);
const float * gyrfilter_current(void);
void magfilter_input(const float val[3]);
const float * magfilter_getCurrent(void);

#endif
