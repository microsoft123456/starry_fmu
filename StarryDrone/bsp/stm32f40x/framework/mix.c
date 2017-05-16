/*
 * File      : mix.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2016-12-29     zoujiachi    rewrite version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>

const static float MIX_MAG_X = 0.743144f;/*cos(42)*/
const static float MIX_MAG_Z = 0.669130f;/*sin42*/

static char* TAG = "MIX";
static float di[3] = {0.0f, 0.0f, 0.0f};
static float accU[3], magU[3];
static float acc_const[3] = {0.0f, 0.0f, 1.0f};
static float acc_constV[3];
static float acc_cross[3];
static float mag_const[3] = {MIX_MAG_X, 0.0f, MIX_MAG_Z};
static float mag_constV[3];
static float mag_cross[3];
static float delta[3];
static float FACTOR_P = 0.6;
static float FACTOR_I = 0.001;
//static float FACTOR_I = 0;

void mix_init(void)
{
	Vector3_Set(di, 0.0f, 0.0f, 0.0f);
	Vector3_Set(acc_const, 0.0f, 0.0f, 1.0f);
	Vector3_Set(mag_const, MIX_MAG_X, 0.0f, MIX_MAG_Z);
}

void Runge_Kutta_1st(quaternion* attitude, quaternion q, float g[3], float deltaT)
{
	attitude->w += (float)0.5*deltaT*( -g[0] * q.x - g[1] * q.y - g[2] * q.z);
	attitude->x += (float)0.5*deltaT*(  g[0] * q.w - g[1] * q.z + g[2] * q.y);
	attitude->y += (float)0.5*deltaT*(  g[0] * q.z + g[1] * q.w - g[2] * q.x);
	attitude->z += (float)0.5*deltaT*( -g[0] * q.y + g[1] * q.x + g[2] * q.w);
}

void mix_gyrAccMag_crossMethod(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float T)
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
	
	quaternion_inv_rotateVector(*q, acc_const, acc_constV);
	Vector3_CrossProduct(acc_cross, accU, acc_constV);
	
	quaternion_inv_rotateVector(*q, mag_const, mag_constV);
	Vector3_CrossProduct(mag_cross, magU, mag_constV);
	
	/* we only need mag value to correct the yaw, so we project the x,y,z axis from
		vehicle to NED axis. By mutiplying mag_cross[2] with this axis in z, we can
		get more accurate value for yaw correction */	
	static float ez[3] = {0,0,1};
	static float vz[3];
	float err[3];
	/* rotate z axis from earth(NED) axis to vehicle axis */
	quaternion_rotateVector(*q, ez, vz);

//	err[0] = acc_cross[0]+mag_cross[2]*fabs(ex[2]);
//	err[1] = acc_cross[1]+mag_cross[2]*fabs(ey[2]);
//	err[2] = acc_cross[2]+mag_cross[2]*fabs(ez[2]);
	err[0] = acc_cross[0]+mag_cross[2]*vz[0];
	err[1] = acc_cross[1]+mag_cross[2]*vz[1];
	err[2] = acc_cross[2]+mag_cross[2]*vz[2];
	
	//TODO: need constrainst the max value of di
	di[0] += err[0];
	if(di[0] >= 1000)
		di[0] = 1000;
	di[1] += err[1];
	if(di[1] >= 1000)
		di[1] = 1000;
	di[2] += err[2];
	if(di[2] >= 1000)
		di[2] = 1000;

	delta[0] = gyr[0] + err[0]*FACTOR_P + di[0]*FACTOR_I;
	delta[1] = gyr[1] + err[1]*FACTOR_P + di[1]*FACTOR_I;
	delta[2] = gyr[2] + err[2]*FACTOR_P + di[2]*FACTOR_I;

	//first order runge-kutta to create quaternion
	Runge_Kutta_1st(q, *q, delta, T);
	quaternion_normalize(q);
}

