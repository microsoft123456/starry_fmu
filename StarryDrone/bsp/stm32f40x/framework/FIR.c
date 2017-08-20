#include <stdint.h>
#include "FIR.h"

// It is an 5th order (length 6) low pass FIR filter with cutoff frequency 5Hz.
// It is for a digital system with a sampling frequency of 0.10kHz.
// Author: Jiachi Zou   Date: 2017
#define FIR_FILTER_LENGTH 6

const float FIRFilterCoefficients[FIR_FILTER_LENGTH] = {
	0.026408, 0.140531, 0.333061, 0.333061, 0.140531, 0.026408
};

int fir_index;
float cir_buffer[FIR_FILTER_LENGTH];

void FIR_Init(void)
{
	int i;
	
	for(i = 0 ; i < FIR_FILTER_LENGTH ; i++){
		cir_buffer[i] = 0.0f;
	}
}

float FIR_Filter(float in)
{
	float out = 0.0f;
	int i;
	
	cir_buffer[fir_index++] = in;
	if(fir_index >= FIR_FILTER_LENGTH)
		fir_index = 0;
	
	for(i = 0 ; i < FIR_FILTER_LENGTH ; i++){
		out += cir_buffer[fir_index] * FIRFilterCoefficients[i];
		if(fir_index != 0){
			fir_index --;
		} else {
			fir_index = FIR_FILTER_LENGTH-1;
		}
	}
	
	return out;
}
