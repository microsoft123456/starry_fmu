/*
 * File      : param.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

PARAM_Def global_param;
PARAM_Def* global_param_t = &global_param;

static const float PI = 3.141592657589793;

rt_err_t load_default_param(PARAM_Def* param_t)
{
	param_t->version = 0;
	
	for(uint8_t i=0 ; i<3 ; i++)
    {
		/* convert dps to rad */
        param_t->gyr_gain[i] = (PI/180.0f);
    }
	param_t->gyr_offset[0] = 1.994650;
    param_t->gyr_offset[1] = -2.291100;
    param_t->gyr_offset[2] = -4.424001;

    param_t->acc_offset[0] = -0.003735;
    param_t->acc_offset[1] = -0.101300;
    param_t->acc_offset[2] = -0.727170;
    param_t->acc_gain[0]   = 0.986756;
    param_t->acc_gain[1]   = 1.010848;
    param_t->acc_gain[2]   = 1.020336;

	param_t->mag_offset[0] = -0.084931;
    param_t->mag_offset[1] = -0.064398;
    param_t->mag_offset[2] = -0.036911;
    param_t->mag_gain[0]   = 2.003295;
    param_t->mag_gain[1]   = 1.944974;
    param_t->mag_gain[2]   = 2.363494;
	
	param_t->initial_attitude.w = 1;
    param_t->initial_attitude.x = 0;
    param_t->initial_attitude.y = 0;
    param_t->initial_attitude.z = 0;
	
    //内环
    param_t->ctl_pid[0].inner_P = 0.1;
    param_t->ctl_pid[0].inner_I = 0;
    param_t->ctl_pid[0].inner_D = 0.08;
    param_t->ctl_pid[1].inner_P = 0.1;    //0.12
    param_t->ctl_pid[1].inner_I = 0;  //0.0001;
    param_t->ctl_pid[1].inner_D = 0.08;    //0.25
    param_t->ctl_pid[2].inner_P = 0.05;
    param_t->ctl_pid[2].inner_I = 0;
    param_t->ctl_pid[2].inner_D = 0.04;
    //外环   ctl_pid
    param_t->ctl_pid[0].outer_P = 0.06;
    param_t->ctl_pid[0].outer_I = 0.003;
    param_t->ctl_pid[0].outer_D = 0;
    param_t->ctl_pid[1].outer_P = 0.06;//0.04 0.06
    param_t->ctl_pid[1].outer_I = 0.003;//0.00002
    param_t->ctl_pid[1].outer_D = 0;
    param_t->ctl_pid[2].outer_P = 0.03;
    param_t->ctl_pid[2].outer_I = 0;
    param_t->ctl_pid[2].outer_P = 0.0015;
	
	param_t->halt_vol = 0;
	param_t->halt_incline_cos = 0.5;	//默认60°停机
	
	return RT_EOK;
}

const PARAM_Def * get_param(void)
{
    return global_param_t;
}

rt_err_t param_init(void)
{
	if(0)
	{
		/* load from sd card */
	}
	else
	{
		/* load default value */
		load_default_param(global_param_t);
	}
	
	return RT_EOK;
}
