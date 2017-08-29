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
#include <math.h>

#define ADDR_CMD_CONVERT_D1			0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2			0x58	/* write to this address to start temperature conversion */

static char *TAG = "Sensor";

static rt_device_t acc_mag_device_t , gyr_device_t , baro_device_t, gps_device_t;
//for debug use
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;

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
//	
//	for(uint8_t i ; i<3 ; i++)
//	{
//		acc[i] = (acc_f[i] + param->acc_offset[i]) * param->acc_gain[i];
//	}
	
	float ofs[3] = { -0.18096, 0.11072, 1.1492};
	float transM[3][3] = {
		{0.98656, -0.00092413, -0.0048647},
		{-0.00092413, 1.0092, 0.013932},
		{-0.0048647, 0.013932, 1.0203}
	};
	
	float ofs_acc[3], rot_acc[3];
	for(uint8_t i=0 ; i<3 ; i++){
		ofs_acc[i] = acc_f[i] - ofs[i];
	}
	for(uint8_t i=0 ; i<3 ; i++){
		acc[i] = ofs_acc[0]*transM[0][i] + ofs_acc[1]*transM[1][i] + ofs_acc[2]*transM[2][i];
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
	
//	for(uint8_t i=0 ; i<3 ; i++)
//	{
//		mag[i] = (mag_f[i] + param->mag_offset[i]) * param->mag_gain[i];
//	}
	
//	mag[0] = 0.905035*mag_f[0] - 0.078528*mag_f[1] - 0.023417*mag_f[2] - 0.025694;
//	mag[1] = 0.938565*mag_f[1] + 0.019469*mag_f[2] - 0.059417;
//	mag[2] = mag_f[2] - 0.089932;
//	mag[0] = 1.024532*mag_f[0] + 0.181527*mag_f[1] - 0.010846*mag_f[2] - 0.067172;
//	mag[1] = 1.284464*mag_f[1] - 0.236356*mag_f[2] + 0.026534;
//	mag[2] = mag_f[2] + 0.069139;
	
//	float ofs[3] = {0.053472, 0.063352, 0.058256};
//	float gain[3] = {0.50187, 0.4447, 0.42454};
//	float rotM[3][3] = {
//		{0.7730,    0.2220,   -0.5942},
//		{0.6049,    0.0244,    0.7960},
//		{-0.1912,    0.9747,    0.1154}
//	};
//	
//	float ofs_mag[3], rot_mag[3];
//	for(uint8_t i=0 ; i<3 ; i++){
//		ofs_mag[i] = mag_f[i] - ofs[i];
//	}
//	for(uint8_t i=0 ; i<3 ; i++){
//		rot_mag[i] = ofs_mag[0]*rotM[0][i] + ofs_mag[1]*rotM[1][i] + ofs_mag[2]*rotM[2][i];
//		mag[i] = rot_mag[i]/gain[i];
//	}

	float ofs[3] = {0.053472, 0.063352, 0.058256};
	float transM[3][3] = {
		{2.1333, -0.17029, 0.030542},
		{-0.17029, 2.2226, 0.039441},
		{0.030542, 0.039441, 2.2408}
	};
	
	float ofs_mag[3], rot_mag[3];
	for(uint8_t i=0 ; i<3 ; i++){
		ofs_mag[i] = mag_f[i] - ofs[i];
	}
	for(uint8_t i=0 ; i<3 ; i++){
		mag[i] = ofs_mag[0]*transM[0][i] + ofs_mag[1]*transM[1][i] + ofs_mag[2]*transM[2][i];
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
				baro_state = S_CONV_2;
		}break;
		case S_CONV_2:
		{
			if(!_baro_is_conv_finish()){	//need 9.04ms to conversion
				err = RT_EBUSY;
			}else{
				err = _baro_read_raw_press();
				if(err == RT_EOK){
					/* directly start D2 conversion */
					err = _baro_trig_conversion(ADDR_CMD_CONVERT_D2);
					if(err == RT_EOK)
						baro_state = S_COLLECT_REPORT;
					else
						baro_state = S_CONV_1;
				}
				else
					baro_state = S_CONV_1;	//if err, restart
			}
		}break;
		case S_COLLECT_REPORT:
		{
			if(!_baro_is_conv_finish()){	//need 9.04ms to conversion
				err = RT_EBUSY;
			}else{
				baro_state = S_CONV_1;
				err = _baro_read_raw_temp();
				if(err == RT_EOK){
					if(rt_device_read(baro_device_t, COLLECT_DATA_POS, (void*)&report_baro, 1)){
						/* start D1 conversion */
						if(_baro_trig_conversion(ADDR_CMD_CONVERT_D1) == RT_EOK)
							baro_state = S_CONV_2;
					}else{
						err = RT_ERROR;
					}
				}
			}
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

struct vehicle_gps_position_s get_gps_position(void)
{
	return gps_position;
}

/* sensor axis is diffirent with attitude(NED) axis */
void sensorAxis2NedAxis(float from[3], float to[3])
{
	to[0] = from[0];
	to[1] = -from[1];
	to[2] = -from[2];
}

int handle_sensor_shell_cmd(int argc, char** argv)
{
	uint8_t sensor_type = 0;
	uint32_t interval = 1000;	//default is 1s
	uint32_t cnt = 0;
	uint8_t raw_data = 0;
	uint8_t no_cali = 0;
	
	if(argc > 1){
		if(strcmp(argv[1], "acc") == 0){
			sensor_type = 1;
		}
		else if(strcmp(argv[1], "mag") == 0){
			sensor_type = 2;
		}
		else if(strcmp(argv[1], "gyr") == 0){
			sensor_type = 3;
		}else{
			printf("unknow parameter:%s\n", argv[1]);
			return 1;
		}
		
		for(uint16_t i = 2 ; i < argc ; i++){
			if(strcmp(argv[i], "-t") == 0){
				i++;
				if(i >= argc){
					printf("wrong cmd format.\n");
					return 2;
				}
				interval = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-n") == 0){
				i++;
				if(i >= argc){
					printf("wrong cmd format.\n");
					return 2;
				}
				cnt = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-r") == 0){
				raw_data = 1;
			}
			if(strcmp(argv[i], "-nc") == 0){
				no_cali = 1;
			}
		}
		
		switch(sensor_type)
		{
			case 1:	//acc
			{
				if(!cnt){
					if(raw_data){
						int16_t raw_acc[3];
						sensor_acc_raw_measure(raw_acc);
						printf("raw acc:%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
					}else if(no_cali){
						float acc[3];
						sensor_acc_measure(acc);
						printf("acc:%f %f %f\n", acc[0], acc[1], acc[2]);
					}else{
						float acc[3];
						sensor_acc_get_calibrated_data(acc);
						printf("cali acc:%f %f %f\n", acc[0], acc[1], acc[2]);
					}
				}else{
					for(uint32_t i = 0 ; i < cnt ; i++){
						if(raw_data){
							int16_t raw_acc[3];
							sensor_acc_raw_measure(raw_acc);
							printf("raw acc:%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
						}else if(no_cali){
							float acc[3];
							sensor_acc_measure(acc);
							printf("acc:%f %f %f\n", acc[0], acc[1], acc[2]);
						}else{
							float acc[3];
							sensor_acc_get_calibrated_data(acc);
							printf("cali acc:%f %f %f\n", acc[0], acc[1], acc[2]);
						}
						rt_thread_delay(interval);
					}
				}
			}break;
			case 2:	//mag
			{
				if(!cnt){
					if(raw_data){
						int16_t raw_mag[3];
						sensor_mag_raw_measure(raw_mag);
						printf("raw mag:%d %d %d\n", raw_mag[0], raw_mag[1], raw_mag[2]);
					}else if(no_cali){
						float mag[3];
						sensor_mag_measure(mag);
						printf("mag:%f %f %f\n", mag[0], mag[1], mag[2]);
					}else{
						float mag[3];
						sensor_mag_get_calibrated_data(mag);
						printf("cali mag:%f %f %f\n", mag[0], mag[1], mag[2]);
					}
				}else{
					for(uint32_t i = 0 ; i < cnt ; i++){
						if(raw_data){
							int16_t raw_mag[3];
							sensor_mag_raw_measure(raw_mag);
							printf("raw mag:%d %d %d\n", raw_mag[0], raw_mag[1], raw_mag[2]);
						}else if(no_cali){
							float mag[3];
							sensor_mag_measure(mag);
							printf("mag:%f %f %f\n", mag[0], mag[1], mag[2]);
						}else{
							float mag[3];
							sensor_mag_get_calibrated_data(mag);
							printf("cali mag:%f %f %f\n", mag[0], mag[1], mag[2]);
						}
						rt_thread_delay(interval);
					}
				}			
			}break;
			case 3:	//gyr
			{
				if(!cnt){
					if(raw_data){
						int16_t raw_gyr[3];
						sensor_gyr_raw_measure(raw_gyr);
						printf("raw gyr:%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
					}else if(no_cali){
						float gyr[3];
						sensor_gyr_measure(gyr);
						printf("gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					}else{
						float gyr[3];
						sensor_gyr_get_calibrated_data(gyr);
						printf("cali gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					}
				}else{
					for(uint32_t i = 0 ; i < cnt ; i++){
						if(raw_data){
							int16_t raw_gyr[3];
							sensor_gyr_raw_measure(raw_gyr);
							printf("raw gyr:%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
						}else if(no_cali){
							float gyr[3];
							sensor_gyr_measure(gyr);
							printf("gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
						}else{
							float gyr[3];
							sensor_gyr_get_calibrated_data(gyr);
							printf("cali gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
						}
						rt_thread_delay(interval);
					}
				}	
			}break;
			default:
				break;
		}
	}
	
	return 0;
}

rt_err_t lsm303d_mag_measure(float mag[3]);
/**************************	INIT FUNC **************************/
rt_err_t device_sensor_init(void)
{
	rt_err_t res;
	
	/* init acc_mag device */
	res = rt_lsm303d_init("spi_d1");
	
	acc_mag_device_t = rt_device_find(ACC_MAG_DEVICE_NAME);
	
	if(acc_mag_device_t == RT_NULL)
	{
		Log.e(TAG, "can't find acc_mag device\r\n");
		return RT_EEMPTY;
	}
	
	rt_device_open(acc_mag_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gyr device */
	res = rt_l3gd20h_init("spi_d2");
	
	gyr_device_t = rt_device_find(GYR_DEVICE_NAME);
	
	if(gyr_device_t == RT_NULL)
	{
		Log.e(TAG, "can't find gyr device\r\n");
		return RT_EEMPTY;
	}
	
	rt_device_open(gyr_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init barometer device */
	baro_state = S_CONV_1;
	
	rt_ms5611_init("spi_d3");
	
	baro_device_t = rt_device_find(BARO_DEVICE_NAME);
	
	if(baro_device_t == RT_NULL)
	{
		Log.e(TAG, "can't find baro device\r\n");
		return RT_EEMPTY;
	}
	
	rt_device_open(baro_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gps device */
	rt_gps_init("uart4" , &gps_position , &satellite_info);
	gps_device_t = rt_device_find(GPS_DEVICE_NAME);
	if(gps_device_t == RT_NULL)
	{
		Log.e(TAG, "can't find gps device\r\n");
		return RT_EEMPTY;
	}
	rt_device_open(gps_device_t , RT_DEVICE_OFLAG_RDWR);
	
	//sensor_get_device_id(GYR_DEVICE_NAME);
	
//	if(rt_rc_init("uart6") == RT_EOK)
//		Log.w(TAG, "rc init ok\r\n");
	
//	while(1)
//	{
//		//example code to read barometer data
//		rt_err_t err;
//		if(sensor_baro_get_state() == S_COLLECT_REPORT){
//			err = sensor_process_baro_state_machine();
//			//get report;
//			if(err == RT_EOK){
//				MS5611_REPORT_Def* report = sensor_baro_get_report();
//				printf("alt:%.2f temp:%.2f press:%.2f\r\n", report->altitude, report->temperature, report->pressure);
//				time_waitMs(200);
//			}
//		}else{
//			err = sensor_process_baro_state_machine();
//		}
//	}

//	while(1)
//	{
//		int16_t mag[3], acc[3];
//		float mag_cali[3], acc_cali[3];
//		sensor_mag_raw_measure(mag);
//		sensor_acc_raw_measure(acc);
//		sensor_mag_get_calibrated_data(mag_cali);
//		sensor_acc_get_calibrated_data(acc_cali);
//		float angle= atan2((double)mag_cali[0], (double)mag_cali[1])*180/3.1415926+180;
//		Log.w(TAG, "mag:%f %f %f mag length:%f angle:%f\n",mag_cali[0],mag_cali[1],mag_cali[2], sqrt(mag_cali[0]*mag_cali[0]+mag_cali[1]*mag_cali[1]+mag_cali[2]*mag_cali[2]), angle);
//		//lsm303d_mag_measure(mag_cali);
//		//Log.w(TAG, "mag %d %d %d mag_c %f %f %f\n", mag[0], mag[1], mag[2], mag_cali[0], mag_cali[1], mag_cali[2]);
//		//Log.w(TAG, "acc %d %d %d acc_c %f %f %f\n", acc[0], acc[1], acc[2], acc_cali[0], acc_cali[1], acc_cali[2]);
//		rt_thread_delay(300);
//	}

//	while(1)
//	{
//		//example code to read gps data
//		if( rt_device_read(gps_device_t, RD_COMPLETED_REPORT, NULL, 1) == RT_EOK){
////			printf("lat:%d lon:%d alt:%d vd:%.2f ve:%.2f vn:%.2f\r\n", gps_position.lat, gps_position.lon, gps_position.alt, 
////				gps_position.vel_d_m_s,gps_position.vel_e_m_s, gps_position.vel_n_m_s);
//			printf("gpspos:%d %d %d gpsv:%f %f %f\r\n", gps_position.lat, gps_position.lon, gps_position.alt, 
//				gps_position.vel_d_m_s,gps_position.vel_e_m_s, gps_position.vel_n_m_s);
//		}
////		if( rt_device_read(gps_device_t, RD_SVINFO, NULL, 1) == RT_EOK){
////			printf("satellite cnt:%d\r\n", satellite_info.count);
////		}
//		
//		time_waitMs(300);
//	}
	
	return res;
}

