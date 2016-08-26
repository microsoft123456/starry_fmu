/*
 * File      : filter.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

static float g_gyr[3];
static float g_mag[3];
static float g_acc[3];

void accfilter_init(void)
{
    for(int i=0;i<3;i++)
    {
        g_acc[i] = 0.0f;
    }
}

void accfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
    {
		g_acc[i] = val[i];
	}
}

const float * accfilter_getCurrent(void)
{
    return g_acc;
}

void gyrfilter_init(void)
{
    for(int i=0;i<3;i++)
        g_gyr[i] = 0;
}

void gyrfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
	{
		g_gyr[i] = val[i];
	}
}

const float * gyrfilter_current(void)
{
    return g_gyr;
}

void magfilter_init(void)
{
    for(int i=0;i<3;i++)
        g_mag[i] = 0;
}

void magfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
        g_mag[i] = val[i];
}

const float * magfilter_getCurrent(void)
{
    return g_mag;
}

rt_err_t filter_init(void)
{
    accfilter_init();   
    gyrfilter_init();  
    magfilter_init();
	
	return RT_EOK;
}
