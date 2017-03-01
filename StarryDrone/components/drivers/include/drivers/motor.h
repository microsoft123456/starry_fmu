/*
 * File      : motor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-02-25     zoujiachi   	the first version
 */

#ifndef _MOTOR_H__
#define _MOTOR_H__

#include <rtthread.h>
#include <rtdevice.h>

#define MOTOR_NUM	4
#define MOTOR_MIN_DC	0.05	/* minimal duty cycle: 1/20=0.05 */
#define MOTOR_MAX_DC	0.1		/* minimal duty cycle: 2/20=0.1 */

#define	MOTOR_CTRL_FREQUENCY	0x01

/* motor device and operations for RT-Thread */
struct rt_device_motor
{
    struct rt_device parent;
    const struct rt_pwm_ops *ops;
};

typedef enum
{
	motor_ch1 = 1,
	motor_ch2 = 2,
	motor_ch3 = 4,
	motor_ch4 = 8,
	motor_ch_all = 0x0F,
}Motor_Channel;

struct rt_pwm_ops
{
    void (*pwm_configure)(struct rt_device *device, rt_base_t frequency);
    void (*pwm_write)(struct rt_device *device, uint8_t chanel, float* duty_cyc);
    int (*pwm_read)(struct rt_device *device, uint8_t chanel, float* buffer);
};

int rt_device_motor_register(const char *name, const struct rt_pwm_ops *ops, void *user_data);

#endif

