/*
 *  internal commands for RT-Thread module shell
 *
 * COPYRIGHT (C) 2013-2015, Shanghai Real-Thread Technology Co., Ltd
 *
 *  This file is part of RT-Thread (http://www.rt-thread.org)
 *  Maintainer: bernard.xiong <bernard.xiong at gmail.com>
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-03-30     Bernard      the first verion for FinSH
 * 2015-08-28     Bernard      Add mkfs command.
 */

#include <rtthread.h>
#include <finsh.h>
#include <shell.h>
//#include "framework/calibration.h"
//#include <rtdevice.h>
#include <string.h>
#include "calibration.h"
#include "px4io_uploader.h"
#include "position.h"

#include "msh.h"

#ifdef FINSH_USING_MSH
#ifdef RT_USING_DFS
#include <dfs_posix.h>

#ifdef DFS_USING_WORKDIR
extern char working_directory[];
#endif

int cmd_ls(int argc, char **argv)
{
    extern void ls(const char *pathname);

    if (argc == 1)
    {
#ifdef DFS_USING_WORKDIR
        ls(working_directory);
#else
        ls("/");
#endif
    }
    else
    {
        ls(argv[1]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_ls, __cmd_ls, List information about the FILEs.);

int cmd_cp(int argc, char **argv)
{
    void copy(const char *src, const char *dst);

    if (argc != 3)
    {
        rt_kprintf("Usage: cp SOURCE DEST\n");
        rt_kprintf("Copy SOURCE to DEST.\n");
    }
    else
    {
        copy(argv[1], argv[2]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cp, __cmd_cp, Copy SOURCE to DEST.);

int cmd_mv(int argc, char **argv)
{
    if (argc != 3)
    {
        rt_kprintf("Usage: mv SOURCE DEST\n");
        rt_kprintf("Rename SOURCE to DEST, or move SOURCE(s) to DIRECTORY.\n");
    }
    else
    {
        int fd;
        char *dest = RT_NULL;

        rt_kprintf("%s => %s\n", argv[1], argv[2]);

        fd = open(argv[2], O_DIRECTORY, 0);
        if (fd >= 0)
        {
            char *src;

            close(fd);

            /* it's a directory */
            dest = (char *)rt_malloc(DFS_PATH_MAX);
            if (dest == RT_NULL)
            {
                rt_kprintf("out of memory\n");
                return -RT_ENOMEM;
            }

            src = argv[1] + rt_strlen(argv[1]);
            while (src != argv[1])
            {
                if (*src == '/') break;
                src --;
            }

            rt_snprintf(dest, DFS_PATH_MAX - 1, "%s/%s", argv[2], src);
        }
        else
        {
            fd = open(argv[2], O_RDONLY, 0);
            if (fd >= 0)
            {
                close(fd);

                unlink(argv[2]);
            }

            dest = argv[2];
        }

        rename(argv[1], dest);
        if (dest != RT_NULL && dest != argv[2]) rt_free(dest);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mv, __cmd_mv, Rename SOURCE to DEST.);

int cmd_cat(int argc, char **argv)
{
    int index;
    extern void cat(const char *filename);

    if (argc == 1)
    {
        rt_kprintf("Usage: cat [FILE]...\n");
        rt_kprintf("Concatenate FILE(s)\n");
        return 0;
    }

    for (index = 1; index < argc; index ++)
    {
        cat(argv[index]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cat, __cmd_cat, Concatenate FILE(s));

int cmd_rm(int argc, char **argv)
{
    int index;

    if (argc == 1)
    {
        rt_kprintf("Usage: rm FILE...\n");
        rt_kprintf("Remove (unlink) the FILE(s).\n");
        return 0;
    }

    for (index = 1; index < argc; index ++)
    {
        unlink(argv[index]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_rm, __cmd_rm, Remove(unlink) the FILE(s).);

#ifdef DFS_USING_WORKDIR
int cmd_cd(int argc, char **argv)
{
    if (argc == 1)
    {
        rt_kprintf("%s\n", working_directory);
    }
    else if (argc == 2)
    {
        chdir(argv[1]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cd, __cmd_cd, Change the shell working directory.);

int cmd_pwd(int argc, char **argv)
{
    rt_kprintf("%s\n", working_directory);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwd, __cmd_pwd, Print the name of the current working directory.);
#endif

int cmd_mkdir(int argc, char **argv)
{
    if (argc == 1)
    {
        rt_kprintf("Usage: mkdir [OPTION] DIRECTORY\n");
        rt_kprintf("Create the DIRECTORY, if they do not already exist.\n");
    }
    else
    {
        mkdir(argv[1], 0);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mkdir, __cmd_mkdir, Create the DIRECTORY.);

int cmd_mkfs(int argc, char **argv)
{
    int result = 0;
    char *type = "elm"; /* use the default file system type as 'fatfs' */

    if (argc == 2)
    {
        result = dfs_mkfs(type, argv[1]);
    }
    else if (argc == 4)
    {
        if (strcmp(argv[1], "-t") == 0)
        {
            type = argv[2];
            result = dfs_mkfs(type, argv[3]);
        }
    }
    else
    {
        rt_kprintf("Usage: mkfs [-t type] device\n");
        return 0;
    }

    if (result != RT_EOK)
    {
        rt_kprintf("mkfs failed, result=%d\n", result);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mkfs, __cmd_mkfs, format disk with file system);

int cmd_echo(int argc, char** argv)
{
	if (argc == 2)
	{
		rt_kprintf("%s\n", argv[1]);
	}
	else if (argc == 3)
	{
		int fd;

		fd = open(argv[2], O_RDWR | O_APPEND | O_CREAT, 0);
		if (fd >= 0)
		{
			write (fd, argv[1], strlen(argv[1]));
			close(fd);
		}
		else
		{
			rt_kprintf("open file:%s failed!\n", argv[2]);
		}
	}
	else
	{
		rt_kprintf("Usage: echo \"string\" [filename]\n");
	}

	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_echo, __cmd_echo, echo string to file);
#endif

#ifdef RT_USING_LWIP
int cmd_ifconfig(int argc, char **argv)
{
    extern void list_if(void);
    extern void set_if(char *netif_name, char *ip_addr, char *gw_addr, char *nm_addr);


    if (argc == 1)
    {
        list_if();
    }
    else if (argc == 5)
    {
        rt_kprintf("config : %s\n", argv[1]);
        rt_kprintf("IP addr: %s\n", argv[2]);
        rt_kprintf("Gateway: %s\n", argv[3]);
        rt_kprintf("netmask: %s\n", argv[4]);
        set_if(argv[1], argv[2], argv[3], argv[4]);
    }
    else
    {
        rt_kprintf("bad parameter! e.g: ifconfig e0 192.168.1.30 192.168.1.1 255.255.255.0\n");
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_ifconfig, __cmd_ifconfig, list the information of network interfaces);

#ifdef RT_LWIP_DNS
#include <lwip/api.h>
#include <lwip/dns.h>
int cmd_dns(int argc, char **argv)
{
    extern void set_dns(char* dns_server);

    if (argc == 1)
    {
        int index;
        struct ip_addr ip_addr;
        for(index=0; index<DNS_MAX_SERVERS; index++)
        {
            ip_addr = dns_getserver(index);
            rt_kprintf("dns server #%d: %s\n", index, ipaddr_ntoa(&(ip_addr)));
        }
    }
    else if (argc == 2)
    {
        rt_kprintf("dns : %s\n", argv[1]);
        set_dns(argv[1]);
    }
    else
    {
        rt_kprintf("bad parameter! e.g: dns 114.114.114.114\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_dns, __cmd_dns, list the information of dns);
#endif

#ifdef RT_LWIP_TCP
int cmd_netstat(int argc, char **argv)
{
    extern void list_tcps(void);

    list_tcps();
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_netstat, __cmd_netstat, list the information of TCP / IP);
#endif
#endif /* RT_USING_LWIP */

int cmd_ps(int argc, char **argv)
{
    extern long list_thread(void);
    extern int list_module(void);

#ifdef RT_USING_MODULE
    if ((argc == 2) && (strcmp(argv[1], "-m") == 0))
        list_module();
    else
#endif
        list_thread();
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_ps, __cmd_ps, List threads in the system.);

int cmd_time(int argc, char **argv)
{
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_time, __cmd_time, Execute command with time.);

#ifdef RT_USING_HEAP
int cmd_free(int argc, char **argv)
{
    extern void list_mem(void);
    extern void list_memheap(void);

#ifdef RT_USING_MEMHEAP_AS_HEAP
    list_memheap();
#else
    list_mem();
#endif
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_free, __cmd_free, Show the memory usage in the system.);
#endif

#endif

/************************ Added by J Zou ************************/
int cmd_calibrate(int argc, char** argv)
{
	struct finsh_shell* shell = finsh_get_shell();
	//char ch;
	
	if(argc != 2)
	{
		rt_kprintf("invalid param\r\n");
		return 1;
	}
	
	if(strcmp(argv[1] , "acc") == 0)
	{
//		rt_kprintf("Calibrate acc:\r\n");
//		
//		/* reset matrix */
//		ResetMatrix();
//		
//		rt_kprintf("forward [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		rt_kprintf("behind [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		rt_kprintf("left [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		rt_kprintf("right [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		rt_kprintf("up [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		rt_kprintf("down [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_acc_data(6);
//		
//		/* calculate result */
//		calibrate_process(ACC_STANDARD_VALUE);

		calibrate_acc_run(shell);
	}
	else if(strcmp(argv[1] , "mag") == 0)
	{
//		rt_kprintf("Calibrate mag:\r\n");
//		
//		/* reset matrix */
//		ResetMatrix();
//		
//		rt_kprintf("forward [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		rt_kprintf("behind [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		rt_kprintf("left [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		rt_kprintf("right [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		rt_kprintf("up [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		rt_kprintf("down [Y/N]\r\n");
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data(6);
//		
//		/* calculate result */
//		calibrate_process(MAG_STANDARD_VALUE);

//		Reset_Cali();
//		
//		for(int i = 0 ; i < 9 ; i++){
//			rt_kprintf("%d point [Y/N]\r\n", i+1);
//			if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//			rt_device_read(shell->device, 0, &ch, 1);
//			if(ch != 'Y' && ch != 'y')
//				return 1;
//			cali_input_mag_data(1);
//		}
//		
//		Calc_Process();

		calibrate_mag_run(shell);
	}
	else if(strcmp(argv[1] , "gyr") == 0)
	{
		calibrate_gyr_run(shell);
//		rt_kprintf("Calibrate gyr:\r\n");
//		calibrate_gyr(200);
	}
	else
	{
		rt_kprintf("invalid param\r\n");
		return 1;
	}
	
	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_calibrate, __cmd_cali, calibrate the acc and mag sensor.);

int cmd_uploader(int argc, char** argv)
{
	px4io_upload();
	
	return 1;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_uploader, __cmd_uploader, upload bin file to px4io.);

int cmd_sethome(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "curpos") == 0)
			set_home_with_current_pos();
		else if(argc == 4){
			unsigned int lon, lat;
			float alt;
			lon = atoi(argv[1]);
			lat = atoi(argv[2]);
			alt = atof(argv[3]);
			rt_kprintf("set home: %d %d %.2f\n", lon, lat, alt);
			set_home(lon, lat, alt);
		}
		
		return 0;
	}
	
	return 1;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sethome, __cmd_sethome, set home with current position.);

int handle_sensor_shell_cmd(int argc, char** argv);
int cmd_sensor(int argc, char** argv)
{
	return handle_sensor_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sensor, __cmd_sensor, get sensor information.);

int handle_motor_shell_cmd(int argc, char** argv);
int cmd_motor(int argc, char** argv)
{
	return handle_motor_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_motor, __cmd_motor, motor operation);

int handle_rc_shell_cmd(int argc, char** argv);
int cmd_rc(int argc, char** argv)
{
	return handle_rc_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_rc, __cmd_rc, rc operation);

int handle_param_shell_cmd(int argc, char** argv);
int cmd_param(int argc, char** argv)
{
	return handle_param_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_param, __cmd_param, configure parameter);

int handle_test_shell_cmd(int argc, char** argv);
int cmd_test(int argc, char** argv)
{
	return handle_test_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_test, __cmd_test, test function);
