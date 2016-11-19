/*
 * File      : gps_sensor.h
 *
 *
 * Change Logs:
 * Date			  Author       	Notes
 * 2016-06-30	  zoujiachi   	the first version
 */
 
#ifndef __GPS_SENSOR_H__
#define __GPS_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>
#include "gps_ubx.h"

#define CONFIGURE_RETRY_MAX		3

rt_err_t rt_gps_init(char* serial_device_name , struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info);

#endif

