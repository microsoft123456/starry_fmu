/*
 * File      : quaternion.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <rtthread.h>
//#include <rtdevice.h>

typedef  struct
{
    float w;
    float x;
    float y;
    float z;
}quaternion;

static inline void quaternion_loadIdentity(quaternion * q);
void quaternion_normalize(quaternion * q);
void quaternion_mult(quaternion * result,const quaternion * left,const quaternion * right);
void quaternion_rotateVector(const quaternion * rotation,const float from[3],float to[3]);
void quaternion_fromTwoVectorRotation(quaternion * result,const float from[3],const float to[3]);
void quaternion_fromFourVectorRotation(quaternion * result,const float from1[3],
const float to1[3],const float from2[3],const float to2[3]);

static inline void quaternion_load_init_attitude(quaternion * q)
{
    q->w = 1;
    q->x = q->y = q->z = 0;
}

#endif
