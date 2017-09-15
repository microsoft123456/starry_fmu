/*
 * File      : pid.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-01     zoujiachi    first version.
 */
 
#include "param.h"
#include "ap_math.h"

static float i_accum[3] = {0,0,0};
static float pre_gyr[3];

uint8_t pid_init(void)
{
	for(int i = 0 ; i < 3 ; i++){
		i_accum[i] = 0;
		pre_gyr[i] = 0;
	}
    return RT_EOK;
}

uint8_t pid_calculate(const float input[3],float output[3], float gyr[3], float dt)
{
    const PARAM_Def *p = get_param();
 
    /* outter ring controls angle */
    float rates_sp[3] = {0};
    for(int i=0;i<3;i++)
    {
		/* outter ring only contains P controller */
        rates_sp[i] = input[i] * p->att_angle_p[i];
    }
    
    /* innter ring controls rates */
    float err_rates[3] = {0};
    for(int i = 0 ; i<3 ; i++)
    {
        err_rates[i] = rates_sp[i] - gyr[i];
		output[i] = 0.0f;
    }
    for(int i=0;i<3;i++)
    {
        output[i] += err_rates[i] * p->att_rate_p[i];

		if(p->att_rate_i[i] > 0.0f){
			i_accum[i] += err_rates[i] * p->att_rate_i[i] * dt;
			
			/* constrain i value between -0.1~0.1 */
			constrain(&i_accum[i], -0.1, 0.1);
			
			output[i] += i_accum[i];
		}
    
        output[i] += (pre_gyr[i] - gyr[i]) * p->att_rate_d[i] / dt;
		/* constrain output */
		constrain(&output[i], -0.5, 0.5);
		/* store last time rates */
		pre_gyr[i] = gyr[i];
    } 
	
	return 0;
}

