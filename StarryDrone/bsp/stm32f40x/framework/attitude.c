/*
 * File      : attitude.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>

#define EVENT_GYR_UPDATE		(1<<0)
#define EVENT_ACC_UPDATE		(1<<1)
#define EVENT_MAG_UPDATE		(1<<2)

#define GYR_UPDATE_INTERVAL		3
#define ACC_UPDATE_INTERVAL		3
#define MAG_UPDATE_INTERVAL		10

//for debug, change to 50 ms
//#define GYR_UPDATE_INTERVAL		100
//#define ACC_UPDATE_INTERVAL		100
//#define MAG_UPDATE_INTERVAL		100

static quaternion drone_attitude;

static struct rt_timer timer_acc;
static struct rt_timer timer_mag;
static struct rt_timer timer_gyr;

struct rt_event event_sensor;

rt_err_t attitude_init(void)
{
	//initialize attitude
	quaternion_load_init_attitude(&drone_attitude);

	return RT_EOK;
}

// get the current attitude
const quaternion * attitude_getAttitude(void)
{
    return &drone_attitude;
}

// acc : unit:m/s²。
void attitude_inputAcc(const float acc[3])
{
    accfilter_input(acc);
}

// gyr : unit:rad/s。
void attitude_inputGyr(const float gyr[3])
{
    gyrfilter_input(gyr);
}

// mag
void attitude_inputMag(const float mag[3])
{
    magfilter_input(mag);
}

/*将陀螺仪和加速度数据进行融合,融合方法为互补滤波*/
void attitude_mixGyrAccMag(void)
{
    //
    // 计算积分间隔，判断是否合理。
    static uint64_t time_pre_us = 0;
    uint64_t time_now_us = time_nowUs();
    int32_t time_interval_us = time_now_us - time_pre_us;   /*积分间隔，单位为us*/
    time_pre_us = time_now_us;
    if(time_interval_us > 1000*1000) // 超过1秒就判为异常，丢弃。
        return;
    float time_interval_s = time_interval_us * (1.0f/1e6f); /*将积分间隔单位转换为s*/
    //
    mix_gyrAccMag_crossMethod(&drone_attitude,gyrfilter_current(),accfilter_getCurrent(),magfilter_getCurrent(),time_interval_s);
}

int32_t acc_gyr_dataIsReady(void)
{
    static uint32_t target = 0;
    uint32_t now = time_nowMs();
    if(target <= now)
    {
        target = now + 10;//每3ms获取一次mpu数据
        //
        return 1;
    }
    return 0;
}

static void timer_acc_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_sensor, EVENT_ACC_UPDATE);
}

static void timer_mag_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_sensor, EVENT_MAG_UPDATE);
}

static void timer_gyr_update(void* parameter)
{
	/* send acc update event */
	rt_event_send(&event_sensor, EVENT_GYR_UPDATE);
}

void attitude_loop(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_ACC_UPDATE | EVENT_MAG_UPDATE | EVENT_GYR_UPDATE;
	float gyr[3] , acc[3] , mag[3];
	
	printf("attitude_loop\r\n");
	
	attitude_init();
	filter_init();
	
	/* create event */
	res = rt_event_init(&event_sensor, "sensor_event", RT_IPC_FLAG_FIFO);
	
	/* register timer event */
	rt_timer_init(&timer_gyr, "timer_gyr",
					timer_gyr_update,
					RT_NULL,
					GYR_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	
	rt_timer_init(&timer_acc, "timer_acc",
					timer_acc_update,
					RT_NULL,
					ACC_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	
	rt_timer_init(&timer_mag, "timer_mag",
					timer_mag_update,
					RT_NULL,
					MAG_UPDATE_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	
	/* start timer */
	rt_timer_start(&timer_gyr);
	/*wait a specific time,letting each event triiger time has time gap*/
	time_waitUs(GYR_UPDATE_INTERVAL*1000/2);
	rt_timer_start(&timer_acc);
	time_waitUs(1000);
	rt_timer_start(&timer_mag);
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_sensor, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								MAG_UPDATE_INTERVAL, &recv_set);
		
		if(res == RT_EOK)
		{
			if(recv_set & EVENT_GYR_UPDATE)
			{
				if(sensor_gyr_get_calibrated_data(gyr) == RT_EOK)
					attitude_inputGyr(gyr);
				
				attitude_mixGyrAccMag();
			}
			
			if(recv_set & EVENT_ACC_UPDATE)
			{
				if(sensor_acc_get_calibrated_data(acc) == RT_EOK)
					attitude_inputAcc(acc);
			}
			
			if(recv_set & EVENT_MAG_UPDATE)
			{
				if(sensor_mag_get_calibrated_data(mag) == RT_EOK)
					attitude_inputMag(mag);
			}
		}
		else
		{
			//some err occur
			rt_kprintf("attitude loop, err:%d\r\n" , res);
		}
		
//		static uint32_t target = 0;
//		uint32_t now = time_nowMs();
//		if(target <= now)
//		{
//			float angle= atan2((double)mag[1], (double)mag[0])*180/3.1415926+180;
//			printf("mag:%.2f  %.2f  %.2f | angle:%f\r\n" , mag[0] , mag[1] , mag[2] , angle);
//			printf("acc %.2f %.2f %.2f\r\n" , acc[0],acc[1],acc[2]);
//			printf("gyr: %.2f %.2f %.2f\r\n" , gyr[0],gyr[1],gyr[2]);
//			target = now + 100;
//			//printf("w:%.2f x:%.2f y:%.2f z:%.2f\r\n" , drone_attitude.w,drone_attitude.x,drone_attitude.y,drone_attitude.z);
//		}
	}
}

