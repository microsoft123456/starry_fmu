/*
 * File      : pid.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include "param.h"

static float pid_pre[3] = {0,0,0};
static float pid_accumulate[3] = {0,0,0};
static float inner_pid_accumulate[3] = {0,0,0};

uint8_t pid_init(void)
{
    return RT_EOK;
}

uint8_t pid_calculate(const float input[3],float output[3], float gyr[3])
{
    const PARAM_Def *p = get_param();
 
    //计算外环
    float outer_output[3] = {0};
    for(int i=0;i<3;i++)
    {
        float p_o = input[i] * p->ctl_pid[i].outer_P; /*比例调节输出分量*/

//        if(baseThrottle>0.2f)
//        {
//            pid_accumulate[i] += input[i]*p->ctl_pid[i].outer_I;    /*积分累加值*/
//        }
//        else
//        {
//           pid_accumulate[i] = 0; 
//        }
		pid_accumulate[i] += input[i]*p->ctl_pid[i].outer_I;
        if(pid_accumulate[i] < -0.05f)  /*限定积分输出分量在-0.1~0.1之间*/
            pid_accumulate[i] = -0.05f;
        if(pid_accumulate[i] > 0.05f)
            pid_accumulate[i] = 0.05f;
            
        float i_o = pid_accumulate[i];  /*积分调节输出分量*/
            
        //外环不需要d
        outer_output[i] = p_o + i_o;    /*分量合成，这里只是作简单的相加*/
    }
    
    //计算内环
    float err_xyz[3] = {0};
    for(int i = 0 ; i<3 ; i++)
    {
        err_xyz[i] = outer_output[i] - gyr[i];
    }
    for(int i=0;i<3;i++)
    {
        float p_o = err_xyz[i] * p->ctl_pid[i].inner_P; /*比例调节输出分量*/
        //if(input[0]*input[0]<0.01 && input[1]*input[1]<0.01 && baseThrottle>0.2)
//        if(baseThrottle>0.2f)
//        {
//            inner_pid_accumulate[i] += err_xyz[i]*p->ctl_pid[i].inner_I;    /*积分累加值*/
//        }
//        else
//        {
//           inner_pid_accumulate[i] = 0; 
//        }
		inner_pid_accumulate[i] += err_xyz[i]*p->ctl_pid[i].inner_I;
        if(inner_pid_accumulate[i] < -0.05f)  /*限定积分输出分量在-0.1~0.1之间*/
            inner_pid_accumulate[i] = -0.05f;
        if(inner_pid_accumulate[i] > 0.05f)
            inner_pid_accumulate[i] = 0.05f;
            
        float i_o = inner_pid_accumulate[i];  /*积分调节输出分量*/
            
        float d_o = (err_xyz[i]-pid_pre[i]) * p->ctl_pid[i].inner_D;
        //
        output[i] = p_o + i_o + d_o;    /*分量合成，这里只是作简单的相加*/
        if(output[i]>0.5){  //输出限幅
            output[i] = 0.5;
        }
		if(output[i]<-0.5){  //输出限幅
            output[i] = -0.5;
        }
        pid_pre[i] = err_xyz[i];  /*存储前一次的输入数据，用于微分输出量计算*/
    } 
	
	return 0;
}

