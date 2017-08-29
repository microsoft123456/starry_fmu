/*
 * File      : pwm.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-06-22     zoujiachi   	the first version
 */
 
#include "stm32f4xx.h"
#include "pwm.h"
#include "log.h"

#define TIM1_FREQUENCY	3000000						// Timer frequency: 3M
#define PWM_DEFAULT_FREQUENCY	50					// pwm default frequqncy: 50Hz
#define PWM_ARR(freq) 	(TIM1_FREQUENCY/freq) 		// CCR reload value, Timer frequency = 3M/60K = 50 Hz

static rt_base_t pwm_freq;
static float TIM_duty_cycle[4] = {0.05, 0.05, 0.05, 0.05};

void pwm_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* GPIOE Configuration: TIM1 CH1 (PE9), TIM1 CH2 (PE11), TIM1 CH3 (PE13) and TIM1 CH4 (PE14) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 

	/* Connect TIM1 pins to AF1 */  
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); 
}

void pwm_timer_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue;
	RCC_ClocksTypeDef  rcc_clocks;

	/* TIM1CLK = 2 * PCLK2  */
    RCC_GetClocksFreq(&rcc_clocks);
	
	/* Compute the prescaler value, TIM1 frequency = 3M Hz */
	PrescalerValue = (uint16_t) ((rcc_clocks.PCLK2_Frequency * 2 / TIM1_FREQUENCY) - 1);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = PWM_ARR(pwm_freq);	//PWM Frequency = 3M/60K = 50 Hz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void stm32_pwm_configure(struct rt_device *device, rt_base_t frequency)
{
	pwm_freq = frequency;
	pwm_timer_init();
}

void stm32_pwm_write(struct rt_device *device, uint8_t chanel, float* duty_cyc)
{
	if(chanel & MOTOR_CH1){
		TIM_SetCompare1(TIM1, PWM_ARR(pwm_freq)*duty_cyc[0]);
		TIM_duty_cycle[0] = duty_cyc[0];
		//Log.console("set ch1 duty:%f\n", TIM_duty_cycle[0]);
	}
	if(chanel & MOTOR_CH2){
		TIM_SetCompare2(TIM1, PWM_ARR(pwm_freq)*duty_cyc[1]);
		TIM_duty_cycle[1] = duty_cyc[1];
		//Log.console("set ch2 duty:%f\n", TIM_duty_cycle[1]);
	}
	if(chanel & MOTOR_CH3){
		TIM_SetCompare3(TIM1, PWM_ARR(pwm_freq)*duty_cyc[2]);
		TIM_duty_cycle[2] = duty_cyc[2];
		//Log.console("set ch3 duty:%f\n", TIM_duty_cycle[2]);
	}
	if(chanel & MOTOR_CH4){
		TIM_SetCompare4(TIM1, PWM_ARR(pwm_freq)*duty_cyc[3]);
		TIM_duty_cycle[3] = duty_cyc[3];
		//Log.console("set ch4 duty:%f\n", TIM_duty_cycle[3]);
	}
}

int stm32_pwm_read(struct rt_device *device, uint8_t chanel, float* buffer)
{
	if(chanel & MOTOR_CH1){
		//Log.console("ch1 duty:%f\n", TIM_duty_cycle[0]);
		buffer[0] = TIM_duty_cycle[0];
	}
	if(chanel & MOTOR_CH2){
		//Log.console("ch2 duty:%f\n", TIM_duty_cycle[1]);
		buffer[1] = TIM_duty_cycle[1];
	}
	if(chanel & MOTOR_CH3){
		//Log.console("ch3 duty:%f\n", TIM_duty_cycle[2]);
		buffer[2] = TIM_duty_cycle[2];
	}
	if(chanel & MOTOR_CH4){
		//Log.console("ch4 duty:%f\n", TIM_duty_cycle[3]);
		buffer[3] = TIM_duty_cycle[3];
	}
	
	return 0;
}

const static struct rt_pwm_ops _stm32_pwm_ops =
{
    stm32_pwm_configure,
    stm32_pwm_write,
    stm32_pwm_read,
};

int stm32_pwm_init(void)
{
	pwm_freq = PWM_DEFAULT_FREQUENCY;
	
	pwm_gpio_init();
	pwm_timer_init();
	
	rt_device_motor_register("motor", &_stm32_pwm_ops, RT_NULL);
	
	return 0;
}
