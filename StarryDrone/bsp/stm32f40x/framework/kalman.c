#include <stdlib.h>
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <sensor.h>
#include "kalman.h"

#define EVENT_POS_UPDATE        (1<<0)
#define EVENT_ACC_UPDATE				(1<<1)

#define POS_UPDATE_INTERVAL     40
#define ACC_UPDATE_INTERVAL			15
#define BARO_UPDATE_INTERVAL    15

#define Q 0.019347
#define R 0.033301

static float drone_global_position[6];

static struct rt_timer timer_pos;
static struct rt_timer timer_baro;
static struct rt_timer timer_acc;

static struct rt_event event_position;

static rt_device_t baro_device_t1, gps_device_t1;
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;

MS5611_REPORT_Def baro_report;

rt_err_t position_init(void)
{
 //Global position initialization
 drone_global_position[0]=0;
 drone_global_position[1]=0;
 drone_global_position[2]=0;
 drone_global_position[3]=0;
 drone_global_position[4]=0;
 drone_global_position[5]=0;

 return RT_EOK;
}

//Get global position
const float* position_getPosition(void)
{
    return drone_global_position;
}

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

static void timer_acc_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_position, EVENT_ACC_UPDATE);
}


#define PI 3.1415926535 
float gaussrand(){
    static float U, V;
    static int phase = 0;
    float Z;

    if(phase == 0)
    {
         U = rand() / (RAND_MAX + 1.0);
         V = rand() / (RAND_MAX + 1.0);
         Z = sqrt(-2.0 * log(U))* sin(2.0 * PI * V);
    }
    else
    {
         Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
    }

    phase = 1 - phase;
    return Z;
}


void kalman2_init(kalman2_state *state, float *init_x, float init_u, float* init_p[2])
{
		float dt_pos = POS_UPDATE_INTERVAL * 1e10-3;
		float dt_acc = ACC_UPDATE_INTERVAL * 1e10-3;
	
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
		//state->u			 ={acceleration}
		state->u = init_u;
		state->B[0]=0.5*dt_acc*dt_acc;
		state->B[0]=dt_acc;
    //state->A       = {{1, 0.01}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = dt_pos;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 1;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = Q;
    state->q[1]    = Q;
    state->r       = R;  /* estimated error convariance */
}



float* kalman2_filter(kalman2_state *state, float u, float *z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1]+
									state->B[0] * state->u;
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1] + 
									state->B[1] * state->u;
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->p[0][0] + state->A[0][1] * state->p[1][0] + 
    				state->A[0][1] * state->p[0][1] + 
    				state->A[0][1] * state->A[0][1] * state->p[1][1] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[0][1] * state->p[0][1] + state->A[1][1] * state->p[1][0];
    state->p[1][1] = state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure[0] - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure[1] - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x;
}

void position_loop(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_POS_UPDATE;
	float gps[3] , acc[3] , baro[3];
	
	printf("position_loop\r\n");
	
	kalman2_state state_x;
	kalman2_state state_y;
	kalman2_state state_z;

	float pos_vel_x[2];
	float pos_vel_y[2];
	float pos_vel_z[2];

	float* result;
	

	//float *init_x = calloc(1, sizeof(int[2]));
	float *init_x = (float*)malloc(2*sizeof(float));
	//float *init_p=calloc(2, sizeof(int[2]));
	float **init_p= (float**)malloc(2*sizeof(float*));
	init_p[0] = malloc(2*sizeof(float));
	init_p[1] = malloc(2*sizeof(float));

	//*init_p=1;
	//*(init_p+3)=1;
	init_p[0][0] = 1;
	init_p[1][1] = 1;
	
	kalman2_init(&state_x, init_x, 0.0f, init_p);
	
	free(init_x);
	free(init_p);
	
	/* create event */
	res = rt_event_init(&event_position, "pos_event", RT_IPC_FLAG_FIFO);
	
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
					
	rt_timer_init(&timer_acc, "timer_acc",
					timer_acc_update,
					RT_NULL,
					ACC_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	

	/* start timer */
	rt_timer_start(&timer_pos);
	rt_timer_start(&timer_baro);
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_position, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);

		
		if(res == RT_EOK)
		{
			if(recv_set & EVENT_POS_UPDATE){
				gps_position = get_gps_position();
				printf("alt:%f\n", baro_report.altitude);
				sensor_acc_get_calibrated_data(acc);

//				//X axis
//				pos_vel_x[0]=gps_position.lat; 
//				pos_vel_x[1]=gps_position.vel_d_m_s;
//				result=kalman2_filter(state_x, acc[0], pos_vel_x);
//				drone_global_position[0]=result[0];
//				drone_global_position[3]=result[1];

//				//Y axis
//				pos_vel_y[0]=gps_position.lat;
//				pos_vel_y[1]=gps_position.vel_d_m_s;
//				result=kalman2_filter(state_y, acc[1], pos_vel_y);
//				drone_global_position[1]=result[0];
//				drone_global_position[4]=result[1];

				//Z axis
				pos_vel_z[0]=baro_report.altitude;
				pos_vel_z[1]=gps_position.vel_d_m_s;
				result=kalman2_filter(&state_z, acc[2], pos_vel_z);
				drone_global_position[2]=result[0];
				drone_global_position[5]=result[1];
				
				static uint32_t now;
				static uint32_t prev = 0;
				now = time_nowMs();
				if(now - prev >= 500){
					prev = now;
					printf("alt:%f v:%f" , drone_global_position[2], drone_global_position[5]);
				}
			}
		}else{
			//some err occur
			rt_kprintf("pos loop, err:%d\r\n" , res);
		}
	}
}