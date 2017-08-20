/*
 * File      : position.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-04-30     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>
#include "FIR.h"

#define EVENT_POS_UPDATE        (1<<0)
#define EVENT_ACC_UPDATE		(1<<1)

#define EVENT_SET_HOME        (1<<0)

#define POS_UPDATE_INTERVAL     100
#define ACC_UPDATE_INTERVAL		15
#define BARO_UPDATE_INTERVAL    15

#define TO_DEGREE(a)	((float)a*1e-7)
#define Deg2Rad(a)		(a*PI/180.0f)

const float EARTH_RADIUS = 6371393;	/* average earth radius, meter */
const float PI = 3.1415926536;

static struct rt_timer timer_pos;
static struct rt_timer timer_baro;

static struct rt_event event_position;
static struct rt_event event_sethome;

static rt_device_t baro_device_t1, gps_device_t1;
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;

static MS5611_REPORT_Def baro_report;

static HOME_Pos home_pos;
kalman2_state state_x;
kalman2_state state_y;
kalman2_state state_z;
static uint8_t home_flag = 0;
static uint8_t init_flag = 0;
static float dt = POS_UPDATE_INTERVAL * 0.001f;
static float cur_alt = 0.0f;

static EKF_Def ekf;

Position_Info pos_info;

static char *TAG = "POS";

static void timer_pos_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_position, EVENT_POS_UPDATE);
}

static void timer_baro_update(void* parameter)
{
	rt_err_t res;
	
	if(sensor_baro_get_state() == S_COLLECT_REPORT){
		res = sensor_process_baro_state_machine();
		//get report;
		if(res == RT_EOK){
			baro_report = *sensor_baro_get_report();
		}
	}else{
		res = sensor_process_baro_state_machine();
	}
}

uint8_t setInitialState(void)
{
	if(!home_flag)
		return 0;
	
//	float cov_ax = 0.002633;
//	//float cov_vx = 0.108990;
//	float cov_vx = 0.1;
//	//float cov_sx = 1364.084034;
//	float cov_sx = 0.1;
	//float cov_ax = 0.529286;
	float cov_ax = 0.1f;
	//float cov_vx = 0.097278;
	float cov_vx = 0.1f;
	//float cov_sy = 6171.600817;
	float cov_sx = 1;
	
	//float cov_ay = 0.529286;
	float cov_ay = 0.1f;
	//float cov_vy = 0.097278;
	float cov_vy = 0.1f;
	//float cov_sy = 6171.600817;
	float cov_sy = 1;
	
	//float cov_az = 0.019341;
	//float cov_az = 0.019341;
	//float cov_az = 0.09;
	float cov_az = 1;
	//float cov_vz = 0.046193;
	float cov_vz = 1;
	//float cov_sz = 0.033301;
	float cov_sz = 1;
	
	/* set initial state to home */
	state_x.x[0] = (float)home_pos.lat;
	state_x.x[1] = 0.0f;
	state_y.x[0] = (float)home_pos.lon;
	//state_y.x[0] = (float)home_pos.lat;
	state_y.x[1] = 0.0f;
	state_z.x[0] = (float)home_pos.alt;
	state_z.x[1] = 0.0f;

	/* Tx is the operator which transfer Delta(x)(meter) to Delta(lat)(1e7 degree) */
	/* Tx = 180/(PI*R), where R is the average radius of earth */
	float Tx = 180.0f/(PI*EARTH_RADIUS)*1e7;
	/* Ax = [ 1  dt*Tx ] */
	/*	    [ 0     1  ] */
	state_x.A[0][0] = 1.0f;
	state_x.A[0][1] = dt*Tx;
	state_x.A[1][0] = 0.0f;
	state_x.A[1][1] = 1.0f;		
	/* Ty is the operator which transfer Delta(y)(meter) to Delta(lon)(1e7 degree) */
	/* Ty = 180/(PI*r), r = R*cos(sita), sita = lat*PI/180, where R is the average radius of earth */
	float sita = Deg2Rad(90.0f-state_x.x[0]*1e-7);
	float r = EARTH_RADIUS*sin(sita);
	float Ty = 180.0f/(PI*r)*1e7;
	
	printf("Tx:%f Ty:%f\n", Tx, Ty);
	
	/* Ay = [ 1  dt*Ty ] */
	/*	    [ 0     1  ] */
	state_y.A[0][0] = 1.0f;
	state_y.A[0][1] = dt*Ty;
	state_y.A[1][0] = 0.0f;
	state_y.A[1][1] = 1.0f;
	/* Az = [ 1  dt] */
	/*	    [ 0  1 ] */
	state_z.A[0][0] = 1.0f;
	state_z.A[0][1] = dt;
	state_z.A[1][0] = 0.0f;
	state_z.A[1][1] = 1.0f;
	
	/* Bx = [ Tx*dt^2/2  dt ]^T */
	state_x.B[0] = Tx*dt*dt/2;
	//state_x.B[0] = 0;
	state_x.B[1] = dt;	
	/* By = [ Ty*dt^2/2  dt ]^T */
	state_y.B[0] = Ty*dt*dt/2;
	//state_y.B[0] = 0;
	state_y.B[1] = dt;	
	/* Bz = [ dt^2/2  dt ]^T */
	state_z.B[0] = dt*dt/2;
	//state_z.B[0] = 0;
	state_z.B[1] = dt;
	/* H = I, don't need to initialize H, because we has ignored H */
	state_x.H[0][0] = state_y.H[0][0] = state_z.H[0][0] = 1.0f;
	state_x.H[0][1] = state_y.H[0][1] = state_z.H[0][1] = 0.0f;
	state_x.H[1][0] = state_y.H[1][0] = state_z.H[1][0] = 0.0f;
	state_x.H[1][1] = state_y.H[1][1] = state_z.H[1][1] = 1.0f;
	/* Q = cov(a)^2 * B * B^T */
	state_x.q[0][0] = Tx*Tx*cov_ax*cov_ax*dt*dt*dt*dt/4;
	state_x.q[0][1] = Tx*cov_ax*cov_ax*dt*dt*dt/2;
	state_x.q[1][0] = Tx*cov_ax*cov_ax*dt*dt*dt/2;
	state_x.q[1][1] = cov_ax*cov_ax*dt*dt;
	state_y.q[0][0] = Ty*Ty*cov_ay*cov_ay*dt*dt*dt*dt/4;
	state_y.q[0][1] = Ty*cov_ay*cov_ay*dt*dt*dt/2;
	state_y.q[1][0] = Ty*cov_ay*cov_ay*dt*dt*dt/2;
	state_y.q[1][1] = cov_ay*cov_ay*dt*dt;
	state_z.q[0][0] = cov_az*cov_az*dt*dt*dt*dt/4;
	state_z.q[0][1] = cov_az*cov_az*dt*dt*dt/2;
	state_z.q[1][0] = cov_az*cov_az*dt*dt*dt/2;
	state_z.q[1][1] = cov_az*cov_az*dt*dt;

//	state_x.q[0][0] = 0;
//	state_x.q[0][1] = 0;
//	state_x.q[1][0] = 0;
//	state_x.q[1][1] = cov_ax*cov_ax*dt*dt;
//	state_y.q[0][0] = 0;
//	state_y.q[0][1] = 0;
//	state_y.q[1][0] = 0;
//	state_y.q[1][1] = cov_ay*cov_ay*dt*dt;
//	state_z.q[0][0] = 0;
//	state_z.q[0][1] = 0;
//	state_z.q[1][0] = 0;
//	state_z.q[1][1] = cov_az*cov_az*dt*dt;
	printf("\nQx:\n");
	printf("%f\n", state_x.q[0][0]);
	printf("%f\n", state_x.q[0][1]);
	printf("%f\n", state_x.q[1][0]);
	printf("%f\n", state_x.q[1][1]);
	printf("Qy:\n");
	printf("%f\n", state_y.q[0][0]);
	printf("%f\n", state_y.q[0][1]);
	printf("%f\n", state_y.q[1][0]);
	printf("%f\n", state_y.q[1][1]);
	printf("Qz:\n");
	printf("%f\n", state_z.q[0][0]);
	printf("%f\n", state_z.q[0][1]);
	printf("%f\n", state_z.q[1][0]);
	printf("%f\n", state_z.q[1][1]);
	/* R = [ cov(s)^2        0    ] */
	/*     [    0        cov(v)^2 ] */
	state_x.r[0][0] = cov_sx*cov_sx;
	state_x.r[0][1] = 0.0f;
	state_x.r[1][0] = 0.0f;
	state_x.r[1][1] = cov_vx*cov_vx;
	state_y.r[0][0] = cov_sy*cov_sy;
	state_y.r[0][1] = 0.0f;
	state_y.r[1][0] = 0.0f;
	state_y.r[1][1] = cov_vy*cov_vy;
	state_z.r[0][0] = cov_sz*cov_sz;
	state_z.r[0][1] = 0.0f;
	state_z.r[1][0] = 0.0f;
	state_z.r[1][1] = cov_vz*cov_vz;
	printf("\nRx:\n");
	printf("%f\n", state_x.r[0][0]);
	printf("%f\n", state_x.r[0][1]);
	printf("%f\n", state_x.r[1][0]);
	printf("%f\n", state_x.r[1][1]);
	printf("Ry:\n");
	printf("%f\n", state_y.r[0][0]);
	printf("%f\n", state_y.r[0][1]);
	printf("%f\n", state_y.r[1][0]);
	printf("%f\n", state_y.r[1][1]);
	printf("Rz:\n");
	printf("%f\n", state_z.r[0][0]);
	printf("%f\n", state_z.r[0][1]);
	printf("%f\n", state_z.r[1][0]);
	printf("%f\n", state_z.r[1][1]);
	
	state_x.p[0][0] = state_x.r[0][0];
	state_x.p[0][1] = state_x.r[0][1];
	state_x.p[1][0] = state_x.r[1][0];
	state_x.p[1][1] = state_x.r[1][1];
	state_y.p[0][0] = state_y.r[0][0];
	state_y.p[0][1] = state_y.r[0][1];
	state_y.p[1][0] = state_y.r[1][0];
	state_y.p[1][1] = state_y.r[1][1];
	state_z.p[0][0] = state_z.r[0][0];
	state_z.p[0][1] = state_z.r[0][1];
	state_z.p[1][0] = state_z.r[1][0];
	state_z.p[1][1] = state_z.r[1][1];
	
	init_flag = 1;
	
	return 1;
}

uint8_t set_home(uint32_t lon, uint32_t lat, float alt)
{
	home_pos.lon = lon;
	home_pos.lat = lat;
	home_pos.alt = alt;
	
	home_flag = 1;
	
	setInitialState();
	
	rt_event_send(&event_sethome, EVENT_SET_HOME);
	
	cur_alt = home_pos.alt;
	
	return home_flag;
}

void set_home_with_current_pos(void)
{
	uint32_t lon, lat;
	float alt;
	gps_position = get_gps_position();
	
//	lon = TO_DEGREE(gps_position.lon);
//	lat = TO_DEGREE(gps_position.lat);
	lon = gps_position.lon;
	lat = gps_position.lat;
	alt = baro_report.altitude;
	
	Log.w(TAG, "set cur home with lon:%d lat:%d alt:%f\n", lon, lat, alt);
	
	set_home(lon, lat, alt);
}

void update_pos_info(Position_Info* p_i, int32_t lat, int32_t lon, int32_t alt, 
						int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz)
{
	p_i->lat = lat;
	p_i->lon = lon;
	p_i->alt = alt;
	p_i->relative_alt = relative_alt;
	p_i->vx = vx;
	p_i->vy = vy;
	p_i->vz = vz;
}

Position_Info get_pos_info(void)
{
	return pos_info;
}

void EKF_Init(EKF_Def* ekf_t, float dT);
void EKF_Update(EKF_Def* ekf_t);

void position_loop(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_POS_UPDATE;
	rt_uint32_t wait_set2 = EVENT_SET_HOME;
	float gps[3] , acc[3] , baro[3];
	float accE[3];
	
	printf("position_loop\r\n");

	float pos_vel_x[2];
	float pos_vel_y[2];
	float pos_vel_z[2];
	
	/* create event */
	res = rt_event_init(&event_position, "pos_event", RT_IPC_FLAG_FIFO);
	res = rt_event_init(&event_sethome, "sethome_event", RT_IPC_FLAG_FIFO);
	
	/* register timer event */
	rt_timer_init(&timer_pos, "timer_pos",
					timer_pos_update,
					RT_NULL,
					POS_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	
	rt_timer_init(&timer_baro, "timer_baro",
					timer_baro_update,
					RT_NULL,
					BARO_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

	/* start timer */
	rt_timer_start(&timer_pos);
	rt_timer_start(&timer_baro);
	
	res = rt_event_recv(&event_sethome, wait_set2, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
	Log.w(TAG, "set home finish \n");
	
	
//	ekf_init(&ekf, dt);
//	/* set initial state to home */
//	ekf.x.element[0][0] = (float)home_pos.lat;
//	ekf.x.element[1][0] = (float)home_pos.lon;
//	ekf.x.element[2][0] = (float)home_pos.alt;
//	ekf.x.element[3][0] = 0.0f;
//	ekf.x.element[4][0] = 0.0f;
//	ekf.x.element[5][0] = 0.0f;
//	init_flag = 1;

	EKF_Init(&ekf, dt);
	/* set initial state to home */
	ekf.x.element[0][0] = (float)home_pos.alt;
	ekf.x.element[1][0] = 0.0f;
	init_flag = 1;
	
	FIR_Init();
	
	float pre_alt = (float)home_pos.alt;
	float calc_v = 0.0f;
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_position, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);

		
		if(res == RT_EOK)
		{
			if(recv_set & EVENT_POS_UPDATE & init_flag){
				/* read sensor value */
				gps_position = get_gps_position();
				sensor_acc_get_calibrated_data(acc);	
				/* transfer acceleration from body grame to global frame */
				quaternion_rotateVector(attitude_getAttitude(), acc, accE);	

				//Log.w(TAG, "acc %f %f %f norm:%f\n", acc[0], acc[1], acc[2], acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
				//Log.w(TAG, "accE %f %f %f norm:%f\n", accE[0], accE[1], accE[2], accE[0]*accE[0]+accE[1]*accE[1]+accE[2]*accE[2]);
				/*remove gravity*/
				accE[2] += 9.8f;
				/*change to NEU axis*/
				accE[2] = -accE[2];
				accE[2] -= 0.1;	//调整偏移
				
//				ekf.u.element[3][0] = accE[0];
//				ekf.u.element[4][0] = accE[1];
//				ekf.u.element[5][0] = accE[2];
////				ekf.u.element[3][0] = 0;
////				ekf.u.element[4][0] = 0;
////				ekf.u.element[5][0] = 0;
//				
//				ekf.z.element[0][0] = gps_position.lat;
//				ekf.z.element[1][0] = gps_position.lon;
////				ekf.z.element[0][0] = 0.0f;
////				ekf.z.element[1][0] = 0.0f;
//				ekf.z.element[2][0] = baro_report.altitude;
//				ekf.z.element[3][0] = gps_position.vel_n_m_s;
//				ekf.z.element[4][0] = gps_position.vel_e_m_s;
//				ekf.z.element[5][0] = gps_position.vel_d_m_s;
////				ekf.z.element[3][0] = 0.0f;
////				ekf.z.element[4][0] = 0.0f;
////				ekf.z.element[5][0] = 0.0f;
//				
//				ekf_update(&ekf);

				//calc_v = calc_v*0.8+(ekf.x.element[0][0]-pre_alt)/ekf.T*0.2;
				calc_v = (ekf.x.element[0][0]-pre_alt)/ekf.T;

				ekf.u.element[0][0] = 0.0f;
				ekf.u.element[1][0] = accE[2];
				
				ekf.z.element[0][0] = baro_report.altitude;
				//ekf.z.element[1][0] = -gps_position.vel_d_m_s;
				ekf.z.element[1][0] = calc_v;
				
				pre_alt = ekf.x.element[0][0];
				EKF_Update(&ekf);
				
				update_pos_info(&pos_info, 0, 0, (int32_t)(ekf.x.element[0][0]*1000.0f), 
						(int32_t)((ekf.x.element[0][0]-home_pos.alt)*1000.0f), 0, 0, (int32_t)(ekf.x.element[1][0]*100));
				
//				/* set observation value */
//				pos_vel_x[0] = gps_position.lat;
//				//pos_vel_x[0] = 0;
//				pos_vel_x[1] = gps_position.vel_n_m_s;
//				//pos_vel_x[1] = 0;
//				pos_vel_y[0] = gps_position.lon; 
//				//pos_vel_y[0] = 0;
//				pos_vel_y[1] = gps_position.vel_e_m_s;
//				//pos_vel_y[1] = 0;
//				pos_vel_z[0] = baro_report.altitude;
//				pos_vel_z[1] = gps_position.vel_d_m_s;
//				
//				//Log.w(TAG, "lat:%f x_v:%f lon:%f y_v:%f\n", pos_vel_x[0], pos_vel_x[1], pos_vel_y[0], pos_vel_y[1]);
//				
//				/* update Ty */
//				float sita = Deg2Rad(90.0f-state_x.x[0]*1e-7);	//这里不是线性的，EKF?
//				float r = EARTH_RADIUS*sin(sita);
//				float Ty = 180.0f/(PI*r)*1e7;
//				/* update Ay, By */
//				state_y.A[0][1] = dt*Ty;
//				state_y.B[0] = dt*dt/2*Ty;
//				
//				/* transfer acceleration from body grame to global frame */
//				quaternion_rotateVector(attitude_getAttitude(), acc, accE);
//				
//				/*remove gravity*/
//				accE[2] += 9.8f;
//				/*change to NEU axis*/
//				accE[2] = -accE[2];
//				
//				kalman2_filter(&state_x, accE[0], pos_vel_x);
//				kalman2_filter(&state_y, accE[1], pos_vel_y);
//				kalman2_filter(&state_z, accE[2], pos_vel_z);
//				
//				update_pos_info(&pos_info, state_x.x[0], state_y.x[0], state_z.x[0]*1000, 
//						(state_z.x[0]-home_pos.alt)*1000, state_x.x[1]*1000, state_y.x[1]*1000, state_z.x[1]*1000);
				
				//Log.w(TAG, "%f %f\n", baro_report.altitude-home_pos.alt, FIR_Filter(baro_report.altitude)-home_pos.alt);
				
				//Log.w(TAG, "%.3f	%.3f	%.3f	%.3f	%.3f	%.3f\n", ekf.x.element[0][0], ekf.z.element[0][0], cur_alt, baro_report.altitude, ekf.x.element[1][0], ekf.z.element[1][0]);
				//Log.w(TAG, "%.3f	%.3f	%.3f\n", baro_report.altitude, calc_v, accE[2]);
				//Log.w(TAG, "%.3f	%.3f	%.3f	%.3f\n", ekf.x.element[0][0], ekf.x.element[1][0], ekf.z.element[0][0], ekf.z.element[1][0]);
				
//				static uint32_t now;
//				static uint32_t prev = 0;
//				now = time_nowMs();
//				if(now - prev >= 300){
//					prev = now;
//					//Log.w(TAG, "%.3f	%.3f	%.3f	%.3f	%.3f	%.3f\n", ekf.x.element[0][0], ekf.x.element[1][0], ekf.z.element[0][0], baro_report.altitude, ekf.z.element[1][0], calc_v);
//				}
			}
		}else{
			//some err occurs
			Log.e(TAG, "pos loop, err:%d\r\n" , res);
		}
	}
}
