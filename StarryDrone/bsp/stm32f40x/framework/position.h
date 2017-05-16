/*
 * File      : position.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-04-30     zoujiachi   	the first version
 */
 
#ifndef __POSITION_H__
#define __POSITION_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef struct
{
	uint32_t 	lon;	/* Lonitude in 1E-7 degrees */
	uint32_t 	lat;	/* Latitude in 1E-7 degrees */
	float		alt;	/* unit: m */
}HOME_Pos;

void position_loop(void *parameter);
void set_home_with_current_pos(void);
uint8_t set_home(uint32_t lon, uint32_t lat, float alt);
	
#endif
