/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdio.h>
#include "board.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif

//void rt_hw_spi_init(void);

//rt_err_t rt_lsm303d_init(char* spi_device_name);

static rt_thread_t tid0;

static char thread_attitude_stack[1024];
struct rt_thread thread_attitude_handle;

static char thread_mavlink_stack[1024];
struct rt_thread thread_mavlink_handle;

static char thread_px4io_stack[1024];
struct rt_thread thread_px4io_handle;

static char thread_control_stack[1024];
struct rt_thread thread_control_handle;

void rt_init_thread_entry(void* parameter)
{
	rt_err_t res;
	
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif
	
	param_init();
	device_led_init();
	device_sensor_init();
	device_mavlink_init();

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
	
	/* create thread */
	res = rt_thread_init(&thread_attitude_handle,
						   "attitude",
						   attitude_loop,
						   RT_NULL,
						   &thread_attitude_stack[0],
						   sizeof(thread_attitude_stack),ATTITUDE_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_attitude_handle);
	
	res = rt_thread_init(&thread_px4io_handle,
						   "px4io",
						   px4io_loop,
						   RT_NULL,
						   &thread_px4io_stack[0],
						   sizeof(thread_px4io_stack),PX4IO_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_px4io_handle);
	
	res = rt_thread_init(&thread_control_handle,
						   "control",
						   control_loop,
						   RT_NULL,
						   &thread_control_stack[0],
						   sizeof(thread_control_stack),CONTROL_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_control_handle);
	
	res = rt_thread_init(&thread_mavlink_handle,
						   "mavlink",
						   mavlink_loop,
						   RT_NULL,
						   &thread_mavlink_stack[0],
						   sizeof(thread_mavlink_stack),MAVLINK_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_mavlink_handle);
	
	TCA62724_blink_control(1);
	
//	while(1)
//	{
//		TCA62724_set_color(LED_RED);
//		time_waitMs(1000);
//		TCA62724_set_color(LED_GREEN);
//		time_waitMs(1000);
//		TCA62724_set_color(LED_BLUE);
//		time_waitMs(1000);
//		TCA62724_set_color(LED_YELLOW);
//		time_waitMs(1000);
//		TCA62724_set_color(LED_WHITE);
//		time_waitMs(1000);
//	}
}

int rt_application_init()
{
    tid0 = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/2, 20);

    if (tid0 != RT_NULL)
        rt_thread_startup(tid0);

    return 0;
}

/*@}*/
