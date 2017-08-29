/*
 * File      : led.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-6    	  zoujiachi   	the first version
 */
 

#include <rtthread.h>
#include <rtdevice.h>
#include "i2c_soft.h"

#define FMU_LED_PIN		43
#define SALVE_ADDR		0x55	//7 bit addr
#define DUTY_OUT0		0x81
#define DUTY_OUT1		0x82
#define DUTY_OUT2		0x83
#define ENABLE_SHDN		0x84
#define WRITE			0
#define READ			1

#define BRIGHT	0x01

rt_device_t _pin_device;
rt_device_t _i2c_device;

static char* TAG = "LED";

uint8_t color_index[][4] = 
{
	{LED_RED 	, 0x00 , 0x00 , BRIGHT},
	{LED_GREEN 	, 0x00 , BRIGHT , 0x00},
	{LED_BLUE 	, BRIGHT , 0x00 , 0x00},
	{LED_YELLOW	, 0x00 , BRIGHT , BRIGHT},
	{LED_WHITE	, BRIGHT , BRIGHT , BRIGHT},
};

uint8_t TCA62724_write_reg(uint8_t duty0, uint8_t duty1, uint8_t duty2)
{
	uint16_t flags = 0x0000;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR);
	
	uint8_t buffer[6];
	buffer[0] = DUTY_OUT0;
	buffer[1] = duty0;
	buffer[2] = DUTY_OUT1;
	buffer[3] = duty1;
	buffer[4] = DUTY_OUT2;
	buffer[5] = duty2;
	
	rt_device_write(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	return 0;
}

uint8_t TCA62724_read_reg(uint8_t reg_val[2])
{
	uint16_t flags = RT_I2C_WR;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR | 1);
	
	uint8_t buffer[2];
	
	rt_device_read(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	reg_val[0] = buffer[0];
	reg_val[1] = buffer[1];
	
	return 0;
}

uint8_t TCA62724_blink_control(uint8_t on_ff)
{
	uint16_t flags = RT_I2C_WR;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR);
	
	uint8_t buffer[2];
	buffer[0] = ENABLE_SHDN;
	
	if(on_ff)
		buffer[1] = 0x03;
	else
		buffer[1] = 0x00;
	
	rt_device_write(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	return 0;
}

uint8_t TCA62724_set_color(LED_COLOR color)
{
	return TCA62724_write_reg(color_index[color][1], color_index[color][2], color_index[color][3]);
}

void led_on(void)
{
	struct rt_device_pin_status pin_sta = {FMU_LED_PIN , 0};
	
	if(_pin_device != RT_NULL)
	{
        _pin_device->write(_pin_device, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

void led_off(void)
{
	struct rt_device_pin_status pin_sta = {FMU_LED_PIN , 1};;
	
	if(_pin_device != RT_NULL)
	{
        _pin_device->write(_pin_device, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

int device_led_init(void)
{
	struct rt_device_pin_mode mode = {FMU_LED_PIN , PIN_MODE_OUTPUT , PIN_OUT_TYPE_OD};
	
    _pin_device = rt_device_find("pin");
	
	if(_pin_device != RT_NULL)
	{
		rt_device_open(_pin_device , RT_DEVICE_OFLAG_RDWR);
        _pin_device->control(_pin_device , 0 , &mode);
    }
	else
	{
		Log.e(TAG, "can not find pin device\n");
		return 1;
	}
	
	device_i2c_init("i2c2");
	
	_i2c_device = rt_device_find("i2c2");
	
	if(_i2c_device != RT_NULL)
	{
		rt_device_open(_i2c_device , RT_DEVICE_OFLAG_RDWR);
    }
	else
	{
		Log.e(TAG, "can not find i2c2 device\n");
		return 1;
	}
	
	return 0;
}
