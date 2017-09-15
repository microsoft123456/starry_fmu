/*
 * File      : filter.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2017-09-06	  zoujiachi	   add butterworth filter and fir filter
 */

#include <math.h>
#include "filter.h"
#include "global.h"
#include "sensor.h"

static float g_gyr[3];
static float g_mag[3];
static float g_acc[3];

static Butter2 _butter_acc[3];
static Butter2 _butter_gyr[3];

void butter_set_cutoff_frequency(Butter2 *butter, float sample_freq, float cutoff_freq)
{
    butter->_cutoff_freq = cutoff_freq;
    if (butter->_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/butter->_cutoff_freq;
    float ohm = tanf(PI/fr);
    float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
    butter->_b0 = ohm*ohm/c;
    butter->_b1 = 2.0f*butter->_b0;
    butter->_b2 = butter->_b0;
    butter->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
    butter->_a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
	
	butter->_delay_element_1 = butter->_delay_element_2 = 0.0f;
}

float butter_filter(Butter2 *butter, float sample)
{
    if (butter->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    float delay_element_0 = sample - butter->_delay_element_1 * butter->_a1 - butter->_delay_element_2 * butter->_a2;
//    if (!isfinite(delay_element_0)) {
//        // don't allow bad values to propagate via the filter
//        delay_element_0 = sample;
//    }
    float output = delay_element_0 * butter->_b0 + butter->_delay_element_1 * butter->_b1 + 
					butter->_delay_element_2 * butter->_b2;
    
    butter->_delay_element_2 = butter->_delay_element_1;
    butter->_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

float butter_reset(Butter2 *butter, float sample)
{
	float dval = sample / (butter->_b0 + butter->_b1 + butter->_b2);
    butter->_delay_element_1 = dval;
    butter->_delay_element_2 = dval;
	
    return butter_filter(butter, sample);
}

void fir_init(FIR* fir, int order, float* coeff, float* buffer_addr)
{
	fir->fir_length = order+1;
	fir->fir_coeff = coeff;
	fir->fir_buffer = buffer_addr;
	fir->fir_index = 0;
	
	for(int i = 0 ; i < fir->fir_length ; i++){
		fir->fir_buffer[i] = 0.0f;
	}
}

float fir_filter(FIR* fir, float sample)
{
	float output = 0.0f;
	
	fir->fir_buffer[fir->fir_index++] = sample;
	if(fir->fir_index >= fir->fir_length)
		fir->fir_index = 0;
	
	for(int i = 0 ; i < fir->fir_length ; i++){
		output += fir->fir_buffer[fir->fir_index] * fir->fir_coeff[i];
		if(fir->fir_index != 0){
			fir->fir_index --;
		} else {
			fir->fir_index = fir->fir_length-1;
		}
	}
	
	return output;
}

///////////////////////////////////////////////////

void accfilter_init(void)
{
    for(int i=0;i<3;i++)
		g_acc[i] = 0.0f;
	
	butter_set_cutoff_frequency(&_butter_acc[0], 1000, 30);
	butter_set_cutoff_frequency(&_butter_acc[1], 1000, 30);
	butter_set_cutoff_frequency(&_butter_acc[2], 1000, 30);
	
	/* set initial data */
	butter_reset(&_butter_acc[0], 0);
	butter_reset(&_butter_acc[1], 0);
	butter_reset(&_butter_acc[2], -9.8f);
}

void accfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
    {
		g_acc[i] = butter_filter(&_butter_acc[i], val[i]);
		//g_acc[i] = val[i];
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
	
	butter_set_cutoff_frequency(&_butter_gyr[0], 1000, 30);
	butter_set_cutoff_frequency(&_butter_gyr[1], 1000, 30);
	butter_set_cutoff_frequency(&_butter_gyr[2], 1000, 30);
	
	/* set initial data */
	butter_reset(&_butter_gyr[0], 0);
	butter_reset(&_butter_gyr[1], 0);
	butter_reset(&_butter_gyr[2], 0);
}

void gyrfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
	{
		g_gyr[i] = butter_filter(&_butter_gyr[i], val[i]);
		//g_gyr[i] = val[i];
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
