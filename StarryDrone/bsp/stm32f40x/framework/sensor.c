/*
 * File      : sensor.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-06-20     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>

#define ADDR_CMD_CONVERT_D1			0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2			0x58	/* write to this address to start temperature conversion */

static rt_device_t acc_mag_device_t , gyr_device_t , baro_device_t;
struct vehicle_gps_position_s gps_position;
struct satellite_info_s satellite_info;

/**************************	ACC API	**************************/
rt_err_t sensor_acc_raw_measure(int16_t acc[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_mag_device_t, 0, (void*)acc, 1);
	
	return r_byte == 1 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_measure(float acc[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_mag_device_t, 2, (void*)acc, 1);
	
	return r_byte == 1 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_get_calibrated_data(float acc[3])
{
	float acc_f[3];
	const PARAM_Def* param = get_param();
	rt_err_t res;
	
	res = sensor_acc_measure(acc_f);
	
	for(uint8_t i ; i<3 ; i++)
	{
		acc[i] = (acc_f[i] + param->acc_offset[i]) * param->acc_gain[i];
	}
	
	return res;
}

/**************************	MAG API	**************************/
rt_err_t sensor_mag_raw_measure(int16_t mag[3])
{
	rt_device_read(acc_mag_device_t, 1, (void*)mag, 1);
	
	return RT_EOK;
}

rt_err_t sensor_mag_measure(float mag[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_mag_device_t, 3, (void*)mag, 1);
	
	return r_byte == 1 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_mag_get_calibrated_data(float mag[3])
{
	float mag_f[3];
	const PARAM_Def* param = get_param();
	rt_err_t res;
	
	res = sensor_mag_measure(mag_f);
	
	for(uint8_t i ; i<3 ; i++)
	{
		mag[i] = (mag_f[i] + param->mag_offset[i]) * param->mag_gain[i];
	}
	
	return res;
}

/**************************	GYR API	**************************/
rt_err_t sensor_gyr_raw_measure(int16_t gyr[3])
{
	rt_device_read(gyr_device_t, 0, (void*)gyr, 1);
	
	return RT_EOK;
}

rt_err_t sensor_gyr_measure(float gyr[3])
{
	rt_size_t r_size;
	r_size = rt_device_read(gyr_device_t, 1, (void*)gyr, 1);
	
	return r_size ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_get_calibrated_data(float gyr[3])
{
	float gyr_dps[3];
	const PARAM_Def* param = get_param();
	rt_err_t res;
	
	res = sensor_gyr_measure(gyr_dps);
	
	for(uint8_t i=0 ; i<3 ; i++)
	{
		gyr[i] = (gyr_dps[i] + param->gyr_offset[i]) * param->gyr_gain[i];
	}
	
	return res;
}

uint8_t sensor_get_device_id(char* device_name)
{
	uint8_t device_id = 0xFF;	//unknown device
	
	if(strcmp(device_name , ACC_MAG_DEVICE_NAME) == 0)
	{
		rt_device_control(acc_mag_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	}else if(strcmp(device_name , GYR_DEVICE_NAME) == 0)
	{
		rt_device_control(gyr_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	}
	
	return device_id;
}

/**************************	BARO API **************************/
static Baro_Machine_State baro_state;
static MS5611_REPORT_Def report_baro;

rt_err_t _baro_trig_conversion(uint8_t addr)
{
	return rt_device_control(baro_device_t, SENSOR_CONVERSION, (void*)&addr);
}

rt_bool_t _baro_is_conv_finish(void)
{
	if(rt_device_control(baro_device_t, SENSOR_IS_CONV_FIN, RT_NULL) == RT_EOK)
	{
		return RT_TRUE;
	}else
	{
		return RT_FALSE;
	}
}

rt_err_t _baro_read_raw_temp(void)
{
	rt_err_t err;
	if(rt_device_read(baro_device_t, RAW_TEMPERATURE_POS, NULL, 1))
		err = RT_EOK;
	else
		err = RT_ERROR;
	
	return err;
}

rt_err_t _baro_read_raw_press(void)
{
	rt_err_t err;
	if(rt_device_read(baro_device_t, RAW_PRESSURE_POS, NULL, 1))
		err = RT_EOK;
	else
		err = RT_ERROR;
	
	return err;
}

/* 
* There are 5 steps to get barometer report
* 1: convert D1
* 2: read pressure raw data
* 3: convert D2
* 4: read temperature raw dara
* 5: compute temperature,pressure,altitute according to prom param.
*/
rt_err_t sensor_process_baro_state_machine(void)
{
	rt_err_t err = RT_ERROR;
	
	switch((uint8_t)baro_state)
	{
		case S_CONV_1:
		{
			err = _baro_trig_conversion(ADDR_CMD_CONVERT_D1);
			if(err == RT_EOK)
				baro_state = S_RAW_PRESS;
		}break;
		case S_RAW_PRESS:
		{
			if(!_baro_is_conv_finish()){
				err = RT_EBUSY;
			}else{
				err = _baro_read_raw_press();
				if(err == RT_EOK)
					baro_state = S_CONV_2;
				else
					baro_state = S_CONV_1;	//if err, restart
			}
		}break;
		case S_CONV_2:
		{
			err = _baro_trig_conversion(ADDR_CMD_CONVERT_D2);
			if(err == RT_EOK)
				baro_state = S_RAW_TEMP;
			else
				baro_state = S_CONV_1;
		}break;
		case S_RAW_TEMP:
		{
			if(!_baro_is_conv_finish()){
				err = RT_EBUSY;
			}else{
				err = _baro_read_raw_temp();
				if(err == RT_EOK)
					baro_state = S_COLLECT_REPORT;
				else
					baro_state = S_CONV_1;
			}
		}break;
		case S_COLLECT_REPORT:
		{
			if(rt_device_read(baro_device_t, COLLECT_DATA_POS, (void*)&report_baro, 1)){
				err = RT_EOK;
			}else{
				err = RT_ERROR;
			}
			baro_state = S_CONV_1;
		}break;
	}
	
	return err;
}

Baro_Machine_State sensor_baro_get_state(void)
{
	return baro_state;
}

MS5611_REPORT_Def* sensor_baro_get_report(void)
{
	return &report_baro;
}

/**************************	INIT FUNC **************************/
rt_err_t device_sensor_init(void)
{
	rt_err_t res;
	
	/* init acc_mag device */
	res = rt_lsm303d_init("spi_d1");
	
	acc_mag_device_t = rt_device_find(ACC_MAG_DEVICE_NAME);
	
	if(acc_mag_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(acc_mag_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gyr device */
	res = rt_l3gd20h_init("spi_d2");
	
	gyr_device_t = rt_device_find(GYR_DEVICE_NAME);
	
	if(gyr_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(gyr_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init barometer device */
	baro_state = S_CONV_1;
	
	rt_ms5611_init("spi_d3");
	
	baro_device_t = rt_device_find(BARO_DEVICE_NAME);
	
	if(baro_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(baro_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gps device */
	rt_gps_init("uart4" , &gps_position , &satellite_info);
	
	//sensor_get_device_id(GYR_DEVICE_NAME);
	
	while(1)
	{
		//example code to read barometer data
		rt_err_t err;
		if(sensor_baro_get_state() == S_COLLECT_REPORT){
			err = sensor_process_baro_state_machine();
			//get report;
			if(err == RT_EOK)
				printf("%.2f %.2f %.2f\r\n" , report_baro.altitude , report_baro.temperature , report_baro.pressure);
		}else{
			err = sensor_process_baro_state_machine();
		}
		time_waitMs(200);
	}
	
//	while(1)
//	{
//		rt_device_read(baro_device_t, COLLECT_DATA_POS, (void*)NULL, 32);
//		time_waitMs(500);
//	}
	
//	while(1)
//	{
//		float acc[3], mag[3];
//		float gyr[3];
//		int16_t raw[3];
//		
//		//sensor_acc_measure(acc);
//		//sensor_mag_measure(mag);
//		
//		//sensor_acc_raw_measure(raw);
//		sensor_gyr_get_calibrated_data(gyr);
//		
//		printf("gyr %.2f %.2f %.2f\r\n" , gyr[0],gyr[1],gyr[2]);
//		//printf("raw acc:%d %d %d\r\n" , raw[0],raw[1],raw[2]);
//		//printf("acc: %.2f %.2f %.2f mag: %.2f %.2f %.2f\r\n" , acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
//		time_waitMs(100);
//	}

//	while(1)
//	{
//		int16_t acc[3], mag[3] , gyr1[3];
//		float gyr2[3];
//		
//		sensor_acc_raw_measure(acc);
//		sensor_mag_raw_measure(mag);
//		
////		sensor_gyr_raw_measure(gyr1);
////		sensor_gyr_rad_measure(gyr2);
//		printf("acc: %d %d %d mag: %d %d %d\r\n" , acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
//		time_waitMs(100);
//	}
	
	return res;
}

