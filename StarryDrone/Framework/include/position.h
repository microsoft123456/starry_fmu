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

typedef struct
{
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
}Position_Info;

void position_loop(void *parameter);
void set_home_with_current_pos(void);
uint8_t set_home(uint32_t lon, uint32_t lat, float alt);
Position_Info get_pos_info(void);
	
#endif
