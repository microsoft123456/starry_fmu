/*
 * File      : calibration.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-20      zoujiachi   	the first version
 */
 
#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include <rtthread.h>
#include <rtdevice.h>

#define ACC_STANDARD_VALUE		9.8f
#define MAG_STANDARD_VALUE		1.0f

void ResetMatrix(void);
void CalcData_Input(double x , double y , double z);
double* calibrate_process(double radius);
void cali_input_acc_data(uint16_t p_num);
void cali_input_mag_data(uint16_t p_num);
rt_err_t calibrate_gyr(uint16_t p_num);

#endif
