/*
 * File      : mix.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2016-12-29     zoujiachi    rewrite version.
 */
 
//#include <rthw.h>
//#include <rtdevice.h>
//#include <rtthread.h>
#include <math.h>
#include "mix.h"

const static float MIX_MAG_X = 0.743144f;/*cos(42)*/
const static float MIX_MAG_Z = 0.669130f;/*sin42*/

static float di[3] = {0.0f, 0.0f, 0.0f};
static float accU[3], magU[3];
//static float acc_const[3] = {0.0f, 0.0f, 1.0f};
/* the gravity is -1, because the acc measures the relative acceleration */
static float acc_const[3] = {0.0f, 0.0f, -1.0f};
static float acc_constV[3];
static float acc_cross[3];
//static float mag_const[3] = {MIX_MAG_X, 0.0f, MIX_MAG_Z};
static float mag_const[3] = {1.0f, 0.0f, 0.0f};
static float mag_constV[3];
static float mag_cross[3];
static float delta[3];
static float FACTOR_P = 0.6;
static float FACTOR_I = 0.001;
//static float FACTOR_I = 0;

static char* TAG = "MIX";

void mix_init(void)
{
	Vector3_Set(di, 0.0f, 0.0f, 0.0f);
	Vector3_Set(acc_const, 0.0f, 0.0f, 1.0f);
	Vector3_Set(mag_const, MIX_MAG_X, 0.0f, MIX_MAG_Z);
}

void Runge_Kutta_1st(quaternion* attitude, quaternion q, float g[3], float dT)
{
	attitude->w += (float)0.5*dT*( -g[0] * q.x - g[1] * q.y - g[2] * q.z);
	attitude->x += (float)0.5*dT*(  g[0] * q.w - g[1] * q.z + g[2] * q.y);
	attitude->y += (float)0.5*dT*(  g[0] * q.z + g[1] * q.w - g[2] * q.x);
	attitude->z += (float)0.5*dT*( -g[0] * q.y + g[1] * q.x + g[2] * q.w);
}

void mix_gyrAcc_crossMethod(quaternion * q,const float gyr[3],const float acc[3],float dT)
{
    float w_q = q->w;
    float x_q = q->x;
    float y_q = q->y;
    float z_q = q->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;

	Vector3_Normalize(accU, acc);
	
	//transfer from body frame to nav frame
	float acc_N[3];
	quaternion_rotateVector(*q, accU, acc_N);
	
	//cross product to calculate diffirence
	Vector3_CrossProduct(acc_cross, acc_N, acc_const);
	
	//create err
	float err_N[3], err_B[3];
	err_N[0] = acc_cross[0];
	err_N[1] = acc_cross[1];
	err_N[2] = acc_cross[2];
	
	//transfer err from nav frame to body frame
	quaternion_inv_rotateVector(*q, err_N, err_B);
	
	//TODO: need constrainst the max value of di
	di[0] += err_B[0];
	if(di[0] >= 100)
		di[0] = 100;
	if(di[0] <= -100)
		di[0] = -100;
	di[1] += err_B[1];
	if(di[1] >= 100)
		di[1] = 100;
	if(di[1] <= -100)
		di[1] = -100;
	di[2] += err_B[2];
	if(di[2] >= 100)
		di[2] = 100;
	if(di[2] <= -100)
		di[2] = -100;

	delta[0] = gyr[0] + err_B[0]*FACTOR_P + di[0]*FACTOR_I;
	delta[1] = gyr[1] + err_B[1]*FACTOR_P + di[1]*FACTOR_I;
	delta[2] = gyr[2] + err_B[2]*FACTOR_P + di[2]*FACTOR_I;

	//first order runge-kutta to create quaternion
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}

void mix_gyrAccMag_crossMethod(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT)
{
    float w_q = q->w;
    float x_q = q->x;
    float y_q = q->y;
    float z_q = q->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;

	Vector3_Normalize(accU, acc);
	Vector3_Normalize(magU, mag);
	
	//transfer from body frame to nav frame
	float acc_N[3], mag_N[3];
	quaternion_rotateVector(*q, accU, acc_N);
	quaternion_rotateVector(*q, magU, mag_N);
	
	//cross product to calculate diffirence
	Vector3_CrossProduct(acc_cross, acc_N, acc_const);
	Vector3_CrossProduct(mag_cross, mag_N, mag_const);
	
	//create err
	float err_N[3], err_B[3];
	err_N[0] = acc_cross[0];
	err_N[1] = acc_cross[1];
	err_N[2] = acc_cross[2] + mag_cross[2];
	
	//transfer err from nav frame to body frame
	quaternion_inv_rotateVector(*q, err_N, err_B);
	
	//TODO: need constrainst the max value of di
	di[0] += err_B[0];
	if(di[0] >= 100)
		di[0] = 100;
	if(di[0] <= -100)
		di[0] = -100;
	di[1] += err_B[1];
	if(di[1] >= 100)
		di[1] = 100;
	if(di[1] <= -100)
		di[1] = -100;
	di[2] += err_B[2];
	if(di[2] >= 100)
		di[2] = 100;
	if(di[2] <= -100)
		di[2] = -100;

	delta[0] = gyr[0] + err_B[0]*FACTOR_P + di[0]*FACTOR_I;
	delta[1] = gyr[1] + err_B[1]*FACTOR_P + di[1]*FACTOR_I;
	delta[2] = gyr[2] + err_B[2]*FACTOR_P + di[2]*FACTOR_I;

	//first order runge-kutta to create quaternion
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}

