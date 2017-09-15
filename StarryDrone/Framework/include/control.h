/*
 * File      : control.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-25     zoujiachi    first version.
 */

#ifndef _CONTROL_H__
#define _CONTROL_H__

//#include <rtthread.h>
//#include <rtdevice.h>
#include <stdint.h>
#include "quaternion.h"

#define MAX_THROTTLE_NUM	4

void control_init(void);
void control_attitude(void);
void control_loop(void *parameter);
void ctrl_set_throttle(float* throttle, uint8_t throttle_num);
void ctrl_unlock_vehicle(void);
void ctrl_lock_vehicle(void);
quaternion calc_target_att(int ctrl_mode, float dT);

#endif
