/*
 * File      : control.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-25     zoujiachi    first version.
 */
 
//#include <rthw.h>
#include <rtdevice.h>

static rt_device_t motor_device_t;
static float throttle_base[MAX_THROTTLE_NUM];

void rc_raw2throttle(uint32_t* raw, float* throttle, uint8_t chan_num)
{
	uint32_t raw_temp;
	for(uint8_t i = 0 ; i<chan_num ; i++){
		raw_temp = raw[i];
		
		if(raw_temp > 2000)
			raw_temp = 2000;
		if(raw_temp < 1000)
			raw_temp = 1000;
		
		throttle[i] = (float)(raw_temp-1000)/1000;
	}
}

void set_throttle_base(float* throttle, uint8_t throttle_num)
{
	if(throttle_num > MAX_THROTTLE_NUM)
		return ;
	
	for(uint8_t i = 0 ; i<throttle_num ; i++)
	{
		throttle_base[i] = throttle[i];
	}
	
	//for motor test
	rt_device_write(motor_device_t, MOTOR_CH_ALL, throttle_base, 4);
}

void control_loop(void *parameter)
{
	motor_device_t = rt_device_find("motor");
	
	if(motor_device_t == RT_NULL)
	{
		printf("err, can not find motor device\n");
		return ;
	}
	
	rt_device_open(motor_device_t , RT_DEVICE_OFLAG_RDWR);
	
	float throttle[4] = {0.0f, 0.1, 0.2, 0.3};
	float read[4];
	rt_base_t freq = 40;
	
	while(1)
	{
		rt_thread_delay(1000);
	}
}
