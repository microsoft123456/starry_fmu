/*
 * File      : ap_math.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __AP_MATH_H__
#define __AP_MATH_H__

#include <rtthread.h>
#include <rtdevice.h>

float math_rsqrt(float number);
float math_vector_length(const float v[3]);
float math_vector_dot(const float left[3],const float right[3]);
void math_vector_cross(float result[3],const float left[3],const float right[3]);
uint16_t math_crc16(uint16_t crc,const void * data,uint16_t len);
void math_itoa(int32_t val,char * str);
const char * math_afromi(int32_t val);

#endif
