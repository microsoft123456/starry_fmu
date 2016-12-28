/*
 * File      : quaternion.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>

void quaternion_normalize(quaternion * q)
{
    float norm_r = math_rsqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w *= norm_r;
    q->x *= norm_r;
    q->y *= norm_r;
    q->z *= norm_r;
}

void quaternion_add(quaternion * result,const quaternion left,const quaternion right)
{
	result->w = left.w + right.w;
	result->x = left.x + right.x;
	result->y = left.y + right.y;
	result->z = left.z + right.z;
}

void quaternion_mult(quaternion * result,const quaternion left,const quaternion right)
{
    result->w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
    result->x = left.x * right.w + left.w * right.x + left.y * right.z - left.z * right.y;
    result->y = left.y * right.w + left.w * right.y + left.z * right.x - left.x * right.z;
    result->z = left.z * right.w + left.w * right.z + left.x * right.y - left.y * right.x;
}

//左乘四元数矩阵
void quaternion_rotateVector(const quaternion rotation,const float from[3],float to[3])
{
    float x2  = rotation.x * 2;
    float y2  = rotation.y * 2;
    float z2  = rotation.z * 2;
    float wx2 = rotation.w * x2;
    float wy2 = rotation.w * y2;
    float wz2 = rotation.w * z2;
    float xx2 = rotation.x * x2;
    float yy2 = rotation.y * y2;
    float zz2 = rotation.z * z2;
    float xy2 = rotation.x * y2;
    float yz2 = rotation.y * z2;
    float xz2 = rotation.z * x2;
    //
    to[0] = from[0]*(1 - yy2 - zz2) + from[1]*(xy2 - wz2)     + from[2]*(xz2 + wy2);
    to[1] = from[0]*(xy2 + wz2)     + from[1]*(1 - xx2 - zz2) + from[2]*(yz2 - wx2);
    to[2] = from[0]*(xz2 - wy2)     + from[1]*(yz2 + wx2)     + from[2]*(1 - xx2 - yy2);
}

//右乘四元数矩阵
void quaternion_inv_rotateVector(const quaternion rotation,const float from[3],float to[3])
{
	float x2  = rotation.x * 2;
    float y2  = rotation.y * 2;
    float z2  = rotation.z * 2;
    float wx2 = rotation.w * x2;
    float wy2 = rotation.w * y2;
    float wz2 = rotation.w * z2;
    float xx2 = rotation.x * x2;
    float yy2 = rotation.y * y2;
    float zz2 = rotation.z * z2;
    float xy2 = rotation.x * y2;
    float yz2 = rotation.y * z2;
    float xz2 = rotation.z * x2;
	//
	to[0] = from[0]*(1 - yy2 - zz2) + from[1]*(xy2 + wz2)     + from[2]*(xz2 - wy2);
	to[1] = from[0]*(xy2 - wz2)     + from[1]*(1 - xx2 - zz2) + from[2]*(yz2 + wx2);
    to[2] = from[0]*(xz2 + wy2)     + from[1]*(yz2 - wx2)     + from[2]*(1 - xx2 - yy2);
}

// calculate quaternion rotate from vector1 to vector2
void quaternion_fromTwoVectorRotation(quaternion * result,const float from[3],const float to[3])
{
    float from_norm = fabsf(from[0]*from[0] + from[1]*from[1] + from[2]*from[2]);
    float to_norm = fabsf(to[0]*to[0] + to[1]*to[1] + to[2]*to[2]);
    //
    from_norm = sqrtf(from_norm);
    to_norm = sqrtf(to_norm);
    float cos_theta = (from[0]*to[0] + from[1]*to[1] + from[2]*to[2]) / (from_norm*to_norm);
    result->w = sqrtf((1.0f + cos_theta) / 2); // cos(theta/2)
    float sin_half_theta = sqrtf((1 - cos_theta) / 2);
    float cross_x = from[1]*to[2] - from[2]*to[1];
    float cross_y = from[2]*to[0] - from[0]*to[2];
    float cross_z = from[0]*to[1] - from[1]*to[0];
    if(cos_theta < 0)
    {
        cross_x = - cross_x;
        cross_y = - cross_y;
        cross_z = - cross_z;
    }
    float sin_half_theta_div_cross_norm = sin_half_theta /
        sqrtf(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
    result->x = cross_x * sin_half_theta_div_cross_norm;
    result->y = cross_y * sin_half_theta_div_cross_norm;
    result->z = cross_z * sin_half_theta_div_cross_norm;
}

//euler[3]: roll pitch yaw
void quaternion_toEuler(const quaternion q, float euler[3])
{
	double ysqr = q.y * q.y;

	// roll (x-axis rotation)
	double t0 = +2.0f * (q.w * q.x + q.y * q.z);
	double t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
	euler[0] = atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0f * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	euler[1] = asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0f * (q.w * q.z + q.x *q.y);
	double t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);  
	euler[2] = atan2(t3, t4);
}

void quaternion_invert(quaternion *q)
{
	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}
