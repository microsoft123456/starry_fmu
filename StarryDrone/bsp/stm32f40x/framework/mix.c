/*
 * File      : mix.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>

const static float MIX_MAG_Y = 0.743144f;/*cos(42)*/
const static float MIX_MAG_Z = 0.669130f;/*sin42*/
//const static float MIX_MAG_Z = 0;/*sin42*/
const static float MIX_MAG_VECTOR[] = {0,0.743144f/*cos(42)*/,-0.669130f/*sin42*/};
const static float MIX_ACC_VECTOR[] = {0,0,1};
const static float MIX_LONGTREM_FACTOR = 0.005;
const static float GRAVITY = 980;
//const static float MIX_GRAVITY = 9.7883;

//Elor 记录初始时飞机的X,Y的磁场强度，作为磁场强度的常量值，进行融合
//float MAG_START_VAL_X , MAG_START_VAL_Y , MAG_START_VAL_Z;

static char* TAG = "MIX";

void mix_init(void);
void mix_gyr(quaternion * attitude,const float gyr[3],float interval);
void mix_AccMag(quaternion * attitude,const float acc[3],const float mag[3]);
void mix_AccMag_steepestDescentMethod(quaternion * attitude,const float acc[3],const float mag[3]);
void mix_Acc_steepestDescentMethod(quaternion * attitude,const float acc[3]);
void mix_gyrAcc_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],float interval);
void mix_gyrAccMag_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],const float mag[3],float interval);


/*************************
函数名：mix_init
功能：mix初始化
参数：无
返回值：无
**************************/
void mix_init(void)
{
}


/*************************
函数名：mix_gyr
功能：传感器初始化
参数：无
返回值：0
**************************/
void mix_gyr(quaternion * attitude,const float gyr[3],float interval)
{
    //
    // 构造增量旋转。
    float delta_x = gyr[0] * interval / 2;
    float delta_y = gyr[1] * interval / 2;
    float delta_z = gyr[2] * interval / 2;
    //
    float q_w = attitude->w;
    float q_x = attitude->x;
    float q_y = attitude->y;
    float q_z = attitude->z;
    //
    // 融合，四元数乘法。
    attitude->w = q_w         - q_x*delta_x - q_y*delta_y - q_z*delta_z;
    attitude->x = q_w*delta_x + q_x         + q_y*delta_z - q_z*delta_y;
    attitude->y = q_w*delta_y - q_x*delta_z + q_y         + q_z*delta_x;
    attitude->z = q_w*delta_z + q_x*delta_y - q_y*delta_x + q_z;
    quaternion_normalize(attitude);
}

// 长期融合，用加速度和磁场方向纠正漂移。
// 梯度下降法，http://blog.sina.com.cn/s/blog_81f1e268010181v3.html
void mix_AccMag_steepestDescentMethod(quaternion * attitude,const float acc[3],const float mag[3])
{
    //
    // 单位化加速度和磁场方向。
    float a_rsqrt = math_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
    float x_a = acc[0] * a_rsqrt;
    float y_a = acc[1] * a_rsqrt;
    float z_a = acc[2] * a_rsqrt;
    //
    float h_rsqrt = math_rsqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float x_h = mag[0] * h_rsqrt;
    float y_h = mag[1] * h_rsqrt;
    float z_h = mag[2] * h_rsqrt;
    //
    float w_q = attitude->w;
    float x_q = attitude->x;
    float y_q = attitude->y;
    float z_q = attitude->z;
    //
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
    //
    float x_da = x_q*z_q_2 - w_q*y_q_2     - x_a;
    float y_da = y_q*z_q_2 + w_q*x_q_2     - y_a;
    float z_da = 1 - x_q*x_q_2 - y_q*y_q_2 - z_a;
    //
    float x_dh = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)     - x_h;
    float y_dh = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)     - y_h;
    float z_dh = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2) - z_h;
    //
    float w_pf =  - x_da*y_q + y_da*x_q + x_dh*(MIX_MAG_Y*z_q - MIX_MAG_Z*y_q) \
            + y_dh*MIX_MAG_Z*x_q - z_dh*MIX_MAG_Y*x_q;
    float x_pf = x_da*z_q + y_da*w_q - z_da*x_q + x_dh*(MIX_MAG_Y*y_q + MIX_MAG_Z*z_q) \
            + y_dh*(MIX_MAG_Z*w_q - MIX_MAG_Y*x_q) - z_dh*(MIX_MAG_Y*w_q + MIX_MAG_Z*x_q);
    float y_pf = - x_da*w_q + y_da*z_q - z_da*y_q + x_dh*(MIX_MAG_Y*x_q - MIX_MAG_Z*w_q) \
            + y_dh*MIX_MAG_Z*z_q + z_dh*(MIX_MAG_Y*z_q - MIX_MAG_Z*y_q);
    float z_pf = x_da*x_q + y_da*y_q + x_dh*(MIX_MAG_Y*w_q + MIX_MAG_Z*x_q) \
            + y_dh *(MIX_MAG_Z*y_q - MIX_MAG_Y*z_q) + z_dh*MIX_MAG_Y*y_q;
    //
    attitude->w -= w_pf * MIX_LONGTREM_FACTOR;
    attitude->x -= x_pf * MIX_LONGTREM_FACTOR;
    attitude->y -= y_pf * MIX_LONGTREM_FACTOR;
    attitude->z -= z_pf * MIX_LONGTREM_FACTOR;
    quaternion_normalize(attitude);
}

// 叉积法融合陀螺和加速度。
void mix_gyrAcc_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],float interval)
{
    const static float FACTOR = 0.001;
    //
    float w_q = attitude->w;
    float x_q = attitude->x;
    float y_q = attitude->y;
    float z_q = attitude->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
    
    //
    // 加速度计的读数，单位化。
    float a_rsqrt = math_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
    float x_aa = acc[0] * a_rsqrt;
    float y_aa = acc[1] * a_rsqrt;
    float z_aa = acc[2] * a_rsqrt;
    //
    // 载体坐标下的重力加速度常量，单位化。     
    float x_ac = x_q*z_q_2 - w_q*y_q_2;         //[x']   [x]
    float y_ac = y_q*z_q_2 + w_q*x_q_2;         //[y']=R*[y]    R是当前姿态的旋转矩阵(论文公式3-4)
    float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;     //[z']   [z]
    //
    // 测量值与常量的叉积。
    float x_ca = y_aa * z_ac - z_aa * y_ac;
    float y_ca = z_aa * x_ac - x_aa * z_ac;
    float z_ca = x_aa * y_ac - y_aa * x_ac;
    //
    // 构造增量旋转。
    float delta_x = gyr[0] * interval / 2 + x_ca * FACTOR;
    float delta_y = gyr[1] * interval / 2 + y_ca * FACTOR;
    float delta_z = gyr[2] * interval / 2 + z_ca * FACTOR;
    //
    // 融合，四元数乘法。
    attitude->w = w_q         - x_q*delta_x - y_q*delta_y - z_q*delta_z;
    attitude->x = w_q*delta_x + x_q         + y_q*delta_z - z_q*delta_y;
    attitude->y = w_q*delta_y - x_q*delta_z + y_q         + z_q*delta_x;
    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;
    quaternion_normalize(attitude);
}

//void mix_gyrAccMag_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],const float mag[3],float interval)
//{
//    const static float FACTOR = 0.001;
//    //
//    float w_q = attitude->w;
//    float x_q = attitude->x;
//    float y_q = attitude->y;
//    float z_q = attitude->z;
//    float x_q_2 = x_q * 2;
//    float y_q_2 = y_q * 2;
//    float z_q_2 = z_q * 2;
//    //
//    // 单位化加速度计的读数。
//    float a_rsqrt = math_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
//    float x_aa = acc[0] * a_rsqrt;
//    float y_aa = acc[1] * a_rsqrt;
//    float z_aa = acc[2] * a_rsqrt;
//    //
//    // 单位化罗盘的读数。
//    float h_rsqrt = math_rsqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
//    float x_hh = mag[0] * h_rsqrt;
//    float y_hh = mag[1] * h_rsqrt;
//    float z_hh = mag[2] * h_rsqrt;
//    //
//    // 载体坐标下的重力加速度常量，已单位化。
//    float x_ac = x_q*z_q_2 - w_q*y_q_2;
//    float y_ac = y_q*z_q_2 + w_q*x_q_2;
//    float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;
//    //
//    // 载体坐标下的地磁场常量，已单位化。
//    float x_hc = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)    ;
//    float y_hc = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)    ;
//    float z_hc = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2);
////    float x_hc = MAG_START_VAL_Y*(x_q*y_q_2 + w_q*z_q_2)     + MAG_START_VAL_X*(1 - y_q*y_q_2 - z_q*z_q_2);
////    float y_hc = MAG_START_VAL_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MAG_START_VAL_X*(x_q*y_q_2 - w_q*z_q_2)    ;
////    float z_hc = MAG_START_VAL_Y*(y_q*z_q_2 - w_q*x_q_2)     + MAG_START_VAL_X*(x_q*z_q_2 + w_q*y_q_2);
//    //
//    // 测量值与常量的叉积。
//    float x_ca = y_aa * z_ac - z_aa * y_ac;
//    float y_ca = z_aa * x_ac - x_aa * z_ac;
//    float z_ca = x_aa * y_ac - y_aa * x_ac;
//    //Elor 这里只需要用磁场纠正Z轴的误差，且需要四轴在水平面的时候才进行纠正
////    float x_ch = y_hh * z_hc - z_hh * y_hc;
////    float y_ch = z_hh * x_hc - x_hh * z_hc;
////    float z_ch = x_hh * y_hc - y_hh * x_hc;
//    float x_ch = 0;
//    float y_ch = 0;
//    float z_ch;
//    if(x_q<=0.03 && y_q<=0.03){
//        z_ch = x_hh * y_hc - y_hh * x_hc;
//        //z_ch = 0;
//        //Log(TAG , "elor %.2f,%.2f" , x_q , y_q);
//    }else{
//        z_ch = 0;
//    }
//    //
//    //
//    // 构造增量旋转。
//    float delta_x = gyr[0] * interval / 2 + (x_ca + x_ch) * FACTOR;
//    float delta_y = gyr[1] * interval / 2 + (y_ca + y_ch) * FACTOR;
//    float delta_z = gyr[2] * interval / 2 + (z_ca + z_ch) * FACTOR;
//    //
//    // 融合，四元数乘法。
//    attitude->w = w_q         - x_q*delta_x - y_q*delta_y - z_q*delta_z;
//    attitude->x = w_q*delta_x + x_q         + y_q*delta_z - z_q*delta_y;
//    attitude->y = w_q*delta_y - x_q*delta_z + y_q         + z_q*delta_x;
//    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;
//    quaternion_normalize(attitude);
//    //printf("w:%.2f , x:%.2f , y:%.2f , z:%%.2f\r\n" , attitude->w , attitude->x , attitude->y , attitude->z);
//}

//互补滤波
void mix_gyrAccMag_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],const float mag[3],float interval)
{
    float w_q = attitude->w;
    float x_q = attitude->x;
    float y_q = attitude->y;
    float z_q = attitude->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
	float acc_sum_square = acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2];
	float mag_sum_square = mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2];

    // 加速度计的读数，单位化。
    float a_rsqrt = math_rsqrt(acc_sum_square);
    float x_aa = acc[0] * a_rsqrt;
    float y_aa = acc[1] * a_rsqrt;
    float z_aa = acc[2] * a_rsqrt;
    
    float h_rsqrt = math_rsqrt(mag_sum_square);
    float x_hh = mag[0] * h_rsqrt;
    float y_hh = mag[1] * h_rsqrt;
    float z_hh = mag[2] * h_rsqrt;
    //
    // 载体坐标下的重力加速度常量，单位化。     
    float x_ac = x_q*z_q_2 - w_q*y_q_2;         //[x']   [x]
    float y_ac = y_q*z_q_2 + w_q*x_q_2;         //[y']=R*[y]    R是当前姿态的旋转矩阵(论文公式3-4)
    float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;     //[z']   [z]
    //
    // 测量值与常量的叉积。
    float x_ca = y_aa * z_ac - z_aa * y_ac;
    float y_ca = z_aa * x_ac - x_aa * z_ac;
    float z_ca = x_aa * y_ac - y_aa * x_ac;
    
    float x_hc = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)    ;
    float y_hc = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)    ;
    float z_hc = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2);
    
    float x_ch = y_hh * z_hc - z_hh * y_hc;
    float y_ch = z_hh * x_hc - x_hh * z_hc;
    float z_ch = x_hh * y_hc - y_hh * x_hc;
    //
    // 构造增量旋转。
	
    float FACTOR_ACC = 0.001f , FACTOR_MAG = 0.001f;
	
	if(sqrt(acc_sum_square) > 1.2*ACC_STANDARD_VALUE)
		FACTOR_ACC = 0.0f;
	else
		FACTOR_ACC = 0.001f;
	
	if(sqrt(mag_sum_square) > 1.2*MAG_STANDARD_VALUE)
		FACTOR_MAG = 0.0f;
	else
		FACTOR_MAG = 0.001f;
	
    float delta_x = gyr[0] * interval / 2 + x_ca * FACTOR_ACC;
    float delta_y = gyr[1] * interval / 2 + y_ca * FACTOR_ACC;
    float delta_z = gyr[2] * interval / 2 + z_ch * FACTOR_MAG;
    float delta_w = sqrt(1-delta_x*delta_x-delta_y*delta_y-delta_z*delta_z);
    //
    // 融合，四元数乘法。
    attitude->w = w_q*delta_w - x_q*delta_x - y_q*delta_y - z_q*delta_z;
    attitude->x = w_q*delta_x + x_q*delta_w + y_q*delta_z - z_q*delta_y;
    attitude->y = w_q*delta_y - x_q*delta_z + y_q*delta_w + z_q*delta_x;
    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q*delta_w;
    quaternion_normalize(attitude);

//    const static float FACTOR = 0.001;
//    //
//    float w_q = attitude->w;
//    float x_q = attitude->x;
//    float y_q = attitude->y;
//    float z_q = attitude->z;
//    float x_q_2 = x_q * 2;
//    float y_q_2 = y_q * 2;
//    float z_q_2 = z_q * 2;
//    
//    //
//    // 加速度计的读数，单位化。
//    float a_rsqrt = math_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
//    float x_aa = acc[0] * a_rsqrt;
//    float y_aa = acc[1] * a_rsqrt;
//    float z_aa = acc[2] * a_rsqrt;
//    
//    float h_rsqrt = math_rsqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
//    float x_hh = mag[0] * h_rsqrt;
//    float y_hh = mag[1] * h_rsqrt;
//    float z_hh = mag[2] * h_rsqrt;
//    //
//    // 载体坐标下的重力加速度常量，单位化。     
//    float x_ac = x_q*z_q_2 - w_q*y_q_2;         //[x']   [x]
//    float y_ac = y_q*z_q_2 + w_q*x_q_2;         //[y']=R*[y]    R是当前姿态的旋转矩阵(论文公式3-4)
//    float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;     //[z']   [z]
//    //
//    // 测量值与常量的叉积。
//    float x_ca = y_aa * z_ac - z_aa * y_ac;
//    float y_ca = z_aa * x_ac - x_aa * z_ac;
//    float z_ca = x_aa * y_ac - y_aa * x_ac;
//    
//    float x_hc = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)    ;
//    float y_hc = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)    ;
//    float z_hc = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2);
//    
//    float x_ch = y_hh * z_hc - z_hh * y_hc;
//    float y_ch = z_hh * x_hc - x_hh * z_hc;
//    float z_ch = x_hh * y_hc - y_hh * x_hc;
//    //
//    // 构造增量旋转。
//    float delta_x = gyr[0] * interval / 2 + x_ca * FACTOR;
//    float delta_y = gyr[1] * interval / 2 + y_ca * FACTOR;
//    float delta_z = gyr[2] * interval / 2 + z_ch * FACTOR;
//    float delta_w = sqrt(1-delta_x*delta_x-delta_y*delta_y-delta_z*delta_z);
//    //
//    // 融合，四元数乘法。
//    attitude->w = w_q*delta_w - x_q*delta_x - y_q*delta_y - z_q*delta_z;
//    attitude->x = w_q*delta_x + x_q*delta_w + y_q*delta_z - z_q*delta_y;
//    attitude->y = w_q*delta_y - x_q*delta_z + y_q*delta_w + z_q*delta_x;
//    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q*delta_w;
//    quaternion_normalize(attitude);
}

void mix_Acc_FullMix(quaternion * attitude , const float acc[3])
{
    const static float FACTOR = 1;
    //
    float w_q = attitude->w;
    float x_q = attitude->x;
    float y_q = attitude->y;
    float z_q = attitude->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
    
    // 加速度计的读数，单位化。
    float a_rsqrt = math_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
    float x_aa = acc[0] * a_rsqrt;
    float y_aa = acc[1] * a_rsqrt;
    float z_aa = acc[2] * a_rsqrt;
    //
    // 载体坐标下的重力加速度常量，单位化。     
    float x_ac = x_q*z_q_2 - w_q*y_q_2;         //[x']   [x]
    float y_ac = y_q*z_q_2 + w_q*x_q_2;         //[y']=R*[y]    R是当前姿态的旋转矩阵(论文公式3-4)
    float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;     //[z']   [z]
    //
    // 测量值与常量的叉积。
    float x_ca = y_aa * z_ac - z_aa * y_ac;
    float y_ca = z_aa * x_ac - x_aa * z_ac;
    float z_ca = x_aa * y_ac - y_aa * x_ac;
    //
    // 构造增量旋转。
    float delta_x = x_ca * FACTOR;
    float delta_y = y_ca * FACTOR;
    float delta_z = 0;
    float delta_w = sqrt(1-delta_x*delta_x-delta_y*delta_y);
    //
    // 融合，四元数乘法。
    attitude->w = w_q*delta_w - x_q*delta_x - y_q*delta_y - z_q*delta_z;
    attitude->x = w_q*delta_x + x_q*delta_w + y_q*delta_z - z_q*delta_y;
    attitude->y = w_q*delta_y - x_q*delta_z + y_q*delta_w + z_q*delta_x;
    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q*delta_w;
    quaternion_normalize(attitude);
}

//融合当前磁场数据,完全融合
void mix_Mag_FullMix(quaternion * attitude , const float mag[3])
{
    const static float FACTOR = 1;
    //
    float w_q = attitude->w;
    float x_q = attitude->x;
    float y_q = attitude->y;
    float z_q = attitude->z;
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
    
    float h_rsqrt = math_rsqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float x_hh = mag[0] * h_rsqrt;
    float y_hh = mag[1] * h_rsqrt;
    float z_hh = mag[2] * h_rsqrt;
    //
    float x_hc = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)    ;
    float y_hc = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)    ;
    float z_hc = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2);
    
    float x_ch = y_hh * z_hc - z_hh * y_hc;
    float y_ch = z_hh * x_hc - x_hh * z_hc;
    float z_ch = x_hh * y_hc - y_hh * x_hc;
    //
    // 构造增量旋转。
    float delta_x = 0;
    float delta_y = 0;
    float delta_z = z_ch * FACTOR;
    float delta_w = sqrt(1-delta_z*delta_z);
    //
    // 融合，四元数乘法。
    attitude->w = w_q*delta_w - x_q*delta_x - y_q*delta_y - z_q*delta_z;
    attitude->x = w_q*delta_x + x_q*delta_w + y_q*delta_z - z_q*delta_y;
    attitude->y = w_q*delta_y - x_q*delta_z + y_q*delta_w + z_q*delta_x;
    attitude->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q*delta_w;
    quaternion_normalize(attitude);
}

