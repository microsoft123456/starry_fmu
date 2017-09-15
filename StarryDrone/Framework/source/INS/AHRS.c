/*
 * File      : AHRS.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2016-12-29     zoujiachi    rewrite version.
 */

#include <math.h>
#include "AHRS.h"

static float accU[3], magU[3];
static float delta[3];
static float FACTOR_P = 2.0;
static float FACTOR_I = 0.005;

void Runge_Kutta_1st(quaternion* attitude, quaternion q, float g[3], float dT)
{
	float halfT = 0.5f*dT;
	attitude->w += halfT * (-g[0] * q.x - g[1] * q.y - g[2] * q.z);
	attitude->x += halfT * ( g[0] * q.w - g[1] * q.z + g[2] * q.y);
	attitude->y += halfT * ( g[0] * q.z + g[1] * q.w - g[2] * q.x);
	attitude->z += halfT * (-g[0] * q.y + g[1] * q.x + g[2] * q.w);
}

void AHRS_reset(quaternion * q, const float acc[3],const float mag[3])
{
	quaternion q1,q2;
	Euler e;
	float to[3];
	float from[3];
	
	to[0] = to[1] = 0.0f;
	to[2] = -1.0f;
	quaternion_fromTwoVectorRotation(&q1, acc, to);
	
	quaternion_rotateVector(q1, mag, from);
	from[2] = 0;
	to[0] = 1;
	to[1] = to[2] = 0;
	quaternion_fromTwoVectorRotation(&q2, from, to);
	
	quaternion_mult(q, q2, q1);
	quaternion_normalize(q);
}

void AHRS_update(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT)
{ 
	float hx, hy, hz, bx, bz;  
	float vx, vy, vz, wx, wy, wz;   
	float ex, ey, ez;  
	static float exInt = 0.0f;
	static float eyInt = 0.0f;
	static float ezInt = 0.0f;

	// auxiliary variables to reduce number of repeated operations   
	float qw_qw_2 = 2.0f*q->w*q->w;  
	float qw_qx_2 = 2.0f*q->w*q->x;  
	float qw_qy_2 = 2.0f*q->w*q->y;  
	float qw_qz_2 = 2.0f*q->w*q->z;  
	float qx_qx_2 = 2.0f*q->x*q->x;  
	float qx_qy_2 = 2.0f*q->x*q->y;  
	float qx_qz_2 = 2.0f*q->x*q->z;  
	float qy_qy_2 = 2.0f*q->y*q->y;  
	float qy_qz_2 = 2.0f*q->y*q->z;  
	float qz_qz_2 = 2.0f*q->z*q->z;

	// normalise the measurements  
	Vector3_Normalize(accU, acc);
	Vector3_Normalize(magU, mag);

	// compute reference direction of magnetic field  
	hx = magU[0]*(1.0f - qy_qy_2 - qz_qz_2) + magU[1]*(qx_qy_2 - qw_qz_2) + magU[2]*(qx_qz_2 + qw_qy_2);  
	hy = magU[0]*(qx_qy_2 + qw_qz_2) + magU[1]*(1.0f - qx_qx_2 - qz_qz_2) + magU[2]*(qy_qz_2 - qw_qx_2);  
	hz = magU[0]*(qx_qz_2 - qw_qy_2) + magU[1]*(qy_qz_2 + qw_qx_2) + magU[2]*(1.0f - qx_qx_2 - qy_qy_2); 
	bx = sqrt((hx*hx) + (hy*hy));  
	bz = hz; 

	// estimated direction of gravity and magnetic field (v and w) 
	// the constant gravity is (0,0,-1), because accelerator measure the relative acceleration
	vx = qw_qy_2 - qx_qz_2;  
	vy = -qw_qx_2 - qy_qz_2;  
	vz = 1.0f - qw_qw_2 - qz_qz_2;   
	wx = bx*(1.0f - qy_qy_2 - qz_qz_2) + bz*(qx_qz_2 - qw_qy_2);  
	wy = bx*(qx_qy_2 - qw_qz_2) + bz*(qw_qx_2 + qy_qz_2);  
	wz = bx*(qw_qy_2 + qx_qz_2) + bz*(1.0f - qx_qx_2 - qy_qy_2); 

	// error is sum ofcross product between reference direction of fields and directionmeasured by sensors   
	ex = (accU[1]*vz - accU[2]*vy) + (magU[1]*wz - magU[2]*wy);  
	ey = (accU[2]*vx - accU[0]*vz) + (magU[2]*wx - magU[0]*wz);  
	ez = (accU[0]*vy - accU[1]*vx) + (magU[0]*wy - magU[1]*wx);  

	// integral error scaled integral gain  
	exInt += ex*FACTOR_I* dT;  
	eyInt += ey*FACTOR_I* dT;  
	ezInt += ez*FACTOR_I* dT;  
	// adjusted gyroscope measurements  
	float delta[3];
	delta[0] = gyr[0] + FACTOR_P*ex + exInt;  
	delta[1] = gyr[1] + FACTOR_P*ey + eyInt;  
	delta[2] = gyr[2] + FACTOR_P*ez + ezInt;  

	// integrate quaternion rate and normalize   
	Runge_Kutta_1st(q, *q, delta, dT);

	// normalise quaternion   	
	quaternion_normalize(q);
}

