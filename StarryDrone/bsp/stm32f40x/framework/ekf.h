/*
 * File      : ekf.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-09     zoujiachi   	the first version
 */
 
#ifndef __EKF_H__
#define __EKF_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef struct
{
	Mat x;		// states
	Mat pre_x;	// predicted states
	Mat u;		// control vector
	//Mat f;		// state function
	Mat F;		// jacobian matrix of f
	Mat P;	
	Mat pre_P;
	Mat Q;
	//Mat h;		// observation function
	Mat H;		// jacobian matrix of h
	Mat R;
	Mat z;		// observation vector
	Mat y;
	Mat S;
	Mat K;		// kalman gain
	float T;	// update interval
}EKF_Def;

void ekf_init(EKF_Def* ekf_t, float dT);
void ekf_update(EKF_Def* ekf_t);

#endif
