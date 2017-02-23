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

#define TIM1_FREQUENCY	3000000
#define PWM_ARR(freq) 	(TIM1_FREQUENCY/freq) // CCR reload value, Timer frequency = 3M/60K = 50 Hz
#define PWM_CCR_MIN(freq) 	(PWM_ARR(freq)*0.05) 		// high pulse 1ms
#define PWM_CCR_MAX(freq) 	(PWM_ARR(freq)*0.1)  		// high pulse 2ms

rt_base_t pwm_freq;
float TIM1_duty_cycle[4] = {0.05, 0.05, 0.05, 0.05};

void pwm_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* GPIOE Configuration: TIM1 CH1 (PE9), TIM1 CH2 (PE11), TIM1 CH3 (PE13) and TIM1 CH4 (PE14) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
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
	TIM_OCInitStructure.TIM_Pulse = PWM_CCR_MIN(pwm_freq);	//initial output minimal pulse
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

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

void pwm_configure(struct rt_device *device, rt_base_t frequency)
{
	pwm_freq = frequency;
	pwm_timer_init();
}

void pwm_write(struct rt_device *device, TIM_Channel chanel, float duty_cyc)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = PWM_ARR(pwm_freq)*duty_cyc;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	if(chanel == tim_ch1){
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	}else if(chanel == tim_ch2){
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	}else if(chanel == tim_ch3){
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	}else if(chanel == tim_ch4){
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	}
	
	TIM1_duty_cycle[chanel] = duty_cyc;
}

float pwm_read(struct rt_device *device, TIM_Channel chanel)
{
	return TIM1_duty_cycle[chanel];
}

int stm32_pwm_init(void)
{
	//pwm_freq = 50;	//init to 50Hz
	//pwm_gpio_init();
	//pwm_timer_init();
	
	return 0;
}
