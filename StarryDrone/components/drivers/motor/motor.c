/*
 * File      : motor.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-02-25     zoujiachi   	the first version
 */
 
#include <drivers/motor.h>

static struct rt_device_motor _hw_motor;

static rt_size_t _motor_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	float throttle_dc[4];
	
	motor->ops->pwm_read(dev, (uint8_t)pos, throttle_dc);
	
	*((float*)buffer) = (throttle_dc[0] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+1) = (throttle_dc[1] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+2) = (throttle_dc[2] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+3) = (throttle_dc[3] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	
    return size;
}

static rt_size_t _motor_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	static float throttle_dc[MOTOR_NUM];
	float throttle;
	
	if(size > MOTOR_NUM){
		printf("unsupport motor num:%ld\n", size);
		return 0;
	}
	
	/* set motor throttle */
	if( pos >= motor_ch1 && pos <= motor_ch_all ){
		if(pos & motor_ch1){
			throttle = *(float*)buffer;
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[0] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & motor_ch2){
			throttle = *((float*)buffer+1);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[1] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & motor_ch3){
			throttle = *((float*)buffer+2);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[2] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & motor_ch4){
			throttle = *((float*)buffer+3);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[3] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		
		motor->ops->pwm_write(dev, (uint8_t)pos, throttle_dc);
	}
	
    return size;
}

static rt_err_t  _motor_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	
	if(cmd == MOTOR_CTRL_FREQUENCY){
		motor->ops->pwm_configure(dev, *(rt_base_t*)args);
	}
	
    return 0;
}


int rt_device_motor_register(const char *name, const struct rt_pwm_ops *ops, void *user_data)
{
    _hw_motor.parent.type         = RT_Device_Class_Miscellaneous;
    _hw_motor.parent.rx_indicate  = RT_NULL;
    _hw_motor.parent.tx_complete  = RT_NULL;

    _hw_motor.parent.init         = RT_NULL;
    _hw_motor.parent.open         = RT_NULL;
    _hw_motor.parent.close        = RT_NULL;
    _hw_motor.parent.read         = _motor_read;
    _hw_motor.parent.write        = _motor_write;
    _hw_motor.parent.control      = _motor_control;

    _hw_motor.ops                 = ops;
    _hw_motor.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_hw_motor.parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
}
