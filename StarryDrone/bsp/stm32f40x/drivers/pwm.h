/*
 * File      : pwm.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-06-22     zoujiachi   	the first version
 */

#ifndef __PWM_H__
#define __PWM_H__

#include <rtdevice.h>

typedef enum
{
	tim_ch1 = 0,
	tim_ch2,
	tim_ch3,
	tim_ch4
}TIM_Channel;

struct rt_pwm_ops
{
    void (*pwm_configure)(struct rt_device *device, rt_base_t frequency);
    void (*pwm_write)(struct rt_device *device, TIM_Channel chanel, float duty_cyc);
    float (*pwm_read)(struct rt_device *device, TIM_Channel chanel);
};

int stm32_pwm_init(void);

#endif


