/*
 * File      : param.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
//#include <rthw.h>
//#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>
#include "global.h"
#include "param.h"
#include "log.h"

PARAM_Def global_param;
PARAM_Def* global_param_t = &global_param;

//static const float PI = 3.141592657589793;

rt_err_t load_default_param(PARAM_Def* param_t)
{
	param_t->version = 0;
	
	for(uint8_t i=0 ; i<3 ; i++)
    {
		/* convert dps to rad */
        param_t->gyr_gain[i] = (PI/180.0f);
    }
	param_t->gyr_offset[0] = 3.199699;
    param_t->gyr_offset[1] = -4.918550;
    param_t->gyr_offset[2] = 6.294048;

    param_t->acc_offset[0] =  0.091815;
    param_t->acc_offset[1] = -0.141828;
    param_t->acc_offset[2] = -0.856508;
    param_t->acc_gain[0]   = 0.979023;
    param_t->acc_gain[1]   = 1.012386;
    param_t->acc_gain[2]   = 1.014212;

	param_t->mag_offset[0] = -0.048217;
    param_t->mag_offset[1] = -0.060532;
    param_t->mag_offset[2] = -0.114581;
    param_t->mag_gain[0]   = 1.810744;
    param_t->mag_gain[1]   = 2.899281;
    param_t->mag_gain[2]   = 2.479019;
	
	param_t->initial_attitude.w = 1;
    param_t->initial_attitude.x = 0;
    param_t->initial_attitude.y = 0;
    param_t->initial_attitude.z = 0;
	
	/* attitude angle control parameter */
	param_t->att_angle_p[0] = 0;
	param_t->att_angle_p[1] = 0.035f;
	param_t->att_angle_p[2] = 0;
	
    /* attitude rates control parameter */
    param_t->att_rate_p[0] = 0;
    param_t->att_rate_i[0] = 0;
    param_t->att_rate_d[0] = 0;
    param_t->att_rate_p[1] = 0.1;
    param_t->att_rate_i[1] = 0;
    param_t->att_rate_d[1] = 0.0015;
    param_t->att_rate_p[2] = 0;
    param_t->att_rate_i[2] = 0;
    param_t->att_rate_d[2] = 0;
	
	param_t->halt_vol = 0;
	param_t->halt_incline_cos = 0.5;	//默认60°停机
	
	return RT_EOK;
}

const PARAM_Def * get_param(void)
{
    return global_param_t;
}

int handle_param_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "get") == 0 && argc == 3){
			if(strcmp(argv[2], "att_pid") == 0){
				Log.console("roll_p:%.3f pitch_p:%.3f yaw_p:%.3f\n",
					global_param_t->att_angle_p[0], global_param_t->att_angle_p[1], global_param_t->att_angle_p[2]);
				Log.console("roll_rate_p:%.3f roll_rate_i:%.3f roll_rate_d:%.3f\n",
					global_param_t->att_rate_p[0], global_param_t->att_rate_i[0], global_param_t->att_rate_d[0]);
				Log.console("pitch_rate_p:%.3f pitch_rate_i:%.3f pitch_rate_d:%.3f\n",
					global_param_t->att_rate_p[1], global_param_t->att_rate_i[1], global_param_t->att_rate_d[1]);
				Log.console("yaw_rate_p:%.3f yaw_rate_i:%.3f yaw_rate_d:%.3f\n",
					global_param_t->att_rate_p[2], global_param_t->att_rate_i[2], global_param_t->att_rate_d[2]);
			}
		}
		if(strcmp(argv[1], "set") == 0 && argc >= 3){
			if(strcmp(argv[2], "att_pid") == 0 && argc == 15){
				global_param_t->att_angle_p[0] = atof(argv[3]);
				global_param_t->att_angle_p[1] = atof(argv[4]);
				global_param_t->att_angle_p[2] = atof(argv[5]);
				
				global_param_t->att_rate_p[0] = atof(argv[6]);
				global_param_t->att_rate_i[0] = atof(argv[7]);
				global_param_t->att_rate_d[0] = atof(argv[8]);
				
				global_param_t->att_rate_p[1] = atof(argv[9]);
				global_param_t->att_rate_i[1] = atof(argv[10]);
				global_param_t->att_rate_d[1] = atof(argv[11]);
				
				global_param_t->att_rate_p[2] = atof(argv[12]);
				global_param_t->att_rate_i[2] = atof(argv[13]);
				global_param_t->att_rate_d[2] = atof(argv[14]);
	
				Log.console("roll_p:%.3f pitch_p:%.3f yaw_p:%.3f\n",
					global_param_t->att_angle_p[0], global_param_t->att_angle_p[1], global_param_t->att_angle_p[2]);
				Log.console("roll_rate_p:%.3f roll_rate_i:%.3f roll_rate_d:%.3f\n",
					global_param_t->att_rate_p[0], global_param_t->att_rate_i[0], global_param_t->att_rate_d[0]);
				Log.console("pitch_rate_p:%.3f pitch_rate_i:%.3f pitch_rate_d:%.3f\n",
					global_param_t->att_rate_p[1], global_param_t->att_rate_i[1], global_param_t->att_rate_d[1]);
				Log.console("yaw_rate_p:%.3f yaw_rate_i:%.3f yaw_rate_d:%.3f\n",
					global_param_t->att_rate_p[2], global_param_t->att_rate_i[2], global_param_t->att_rate_d[2]);
			}
		}
	}
	
	return 0;
}

uint8_t param_init(void)
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
	
	return 0;
}
