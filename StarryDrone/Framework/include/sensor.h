/*
 * File      : sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-20      zoujiachi   	the first version
 */
 
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <rtthread.h>
#include <stdint.h>
#include "ms5611.h"

#define ACC_MAG_DEVICE_NAME		"acc_mag"
#define GYR_DEVICE_NAME			"gyr"
#define BARO_DEVICE_NAME		"baro"
#define GPS_DEVICE_NAME			"gps"

#define RAW_TEMPERATURE_POS			0
#define RAW_PRESSURE_POS			1
#define COLLECT_DATA_POS			2

/* control cmd */

//common cmd
#define SENSOR_GET_DEVICE_ID		0x00

//acc,mag cmd
#define SENSOR_SET_ACC_RANGE		0x01
#define SENSOR_SET_ACC_SAMPLERATE	0x02
#define SENSOR_SET_MAG_RANGE		0x03
#define SENSOR_SET_MAG_SAMPLERATE	0x04

//gyr cmd
#define SENSOR_SET_GYR_RANGE		0x20

//baro cmd
#define SENSOR_CONVERSION			0x30
#define SENSOR_IS_CONV_FIN			0x31

typedef enum
{
	S_CONV_1 = 0,
	S_RAW_PRESS,
	S_CONV_2,
	S_RAW_TEMP,
	S_COLLECT_REPORT
}Baro_Machine_State;


rt_err_t device_sensor_init(void);

/* acc API */
rt_err_t sensor_acc_raw_measure(int16_t acc[3]);
rt_err_t sensor_acc_measure(float acc[3]);
rt_err_t sensor_acc_get_calibrated_data(float acc[3]);

/* mag API */
rt_err_t sensor_mag_raw_measure(int16_t mag[3]);
rt_err_t sensor_mag_measure(float mag[3]);
rt_err_t sensor_mag_get_calibrated_data(float mag[3]);

/* gyr API */
rt_err_t sensor_gyr_raw_measure(int16_t gyr[3]);
rt_err_t sensor_gyr_measure(float gyr[3]);
rt_err_t sensor_gyr_get_calibrated_data(float gyr[3]);

/* barometer API */
Baro_Machine_State sensor_baro_get_state(void);
MS5611_REPORT_Def* sensor_baro_get_report(void);
rt_err_t sensor_process_baro_state_machine(void);

/* gps API */
struct vehicle_gps_position_s get_gps_position(void);

/* common api */
void sensorAxis2NedAxis(float from[3], float to[3]);

#endif
