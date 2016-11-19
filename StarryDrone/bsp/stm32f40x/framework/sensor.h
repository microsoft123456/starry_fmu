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
#include <rtdevice.h>

#define ACC_MAG_DEVICE_NAME		"acc_mag"
#define GYR_DEVICE_NAME			"gyr"
#define BARO_DEVICE_NAME		"baro"

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
rt_err_t sensor_acc_raw_measure(int16_t acc[3]);
rt_err_t sensor_mag_raw_measure(int16_t mag[3]);
rt_err_t sensor_gyr_raw_measure(int16_t gyr[3]);
rt_err_t sensor_acc_measure(float acc[3]);
rt_err_t sensor_mag_measure(float mag[3]);
rt_err_t sensor_gyr_measure(float gyr[3]);
rt_err_t sensor_acc_get_calibrated_data(float acc[3]);
rt_err_t sensor_mag_get_calibrated_data(float mag[3]);
rt_err_t sensor_gyr_get_calibrated_data(float gyr[3]);
rt_err_t sensor_baro_trig_conversion(uint8_t addr);
rt_bool_t sensor_baro_conversion_finish(void);
//rt_err_t sensor_baro_read_raw_temp(uint32_t* raw);
//rt_err_t sensor_baro_read_raw_press(uint32_t* raw);
//rt_err_t sensor_baro_read_temperature(int32_t* temp);
//rt_err_t sensor_baro_read_pressure(int32_t* pressure);

#endif
