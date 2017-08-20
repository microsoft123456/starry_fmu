/*
 * File      : ekf.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-09     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>

#define STATE_NUM		(6)
static const float PI = 3.141592658;
const float R0 = 6371393.0f;

#define Deg2Rad(a)		(a*PI/180.0f)

Mat temp_mat1, temp_mat2, temp_mat3;
Mat temp_mat4;
static float Tx = 180.0f*1e7/(PI*R0);

void ekf_init(EKF_Def* ekf_t, float dT)
{
	MatCreate(&ekf_t->x, STATE_NUM, 1);
	MatCreate(&ekf_t->pre_x, STATE_NUM, 1);
	MatCreate(&ekf_t->u, STATE_NUM, 1);
	MatCreate(&ekf_t->F, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->P, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->pre_P, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->Q, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->H, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->R, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->z, STATE_NUM, 1);
	MatCreate(&ekf_t->y, STATE_NUM, 1);
	MatCreate(&ekf_t->S, STATE_NUM, STATE_NUM);
	MatCreate(&ekf_t->K, STATE_NUM, STATE_NUM);
	
	MatCreate(&temp_mat1, STATE_NUM, STATE_NUM);
	MatCreate(&temp_mat2, STATE_NUM, STATE_NUM);
	MatCreate(&temp_mat3, STATE_NUM, STATE_NUM);
	MatCreate(&temp_mat4, STATE_NUM, 1);
	
	ekf_t->T = dT;
	
	/* init F and H */
	MatEye(&ekf_t->F);
	ekf_t->F.element[0][3] = Tx*ekf_t->T;
	ekf_t->F.element[2][5] = -ekf_t->T;
	
	MatEye(&ekf_t->H);
	
	/* Q = (B*[cov_aN cov_aE cov_aD]') * (B*[cov_aN cov_aE cov_aD]')' */
	float cov_1 = 100.0f;
	float cov_2 = 100.0f;
	//float cov_3 = 0.1f*ekf_t->T;
	float cov_3 = 1.0;
	float cov_aN = 0.6;
	float cov_aE = 0.6;
	float cov_aD = 0.6;
	MatZeros(&ekf_t->Q);
	ekf_t->Q.element[0][0] = cov_1*cov_1;
	ekf_t->Q.element[1][1] = cov_2*cov_2;
	ekf_t->Q.element[2][2] = cov_3*cov_3;
	ekf_t->Q.element[3][3] = cov_aN*cov_aN;
	ekf_t->Q.element[4][4] = cov_aE*cov_aE;
	ekf_t->Q.element[5][5] = cov_aD*cov_aD;
	//MatEye(&ekf_t->Q);
	
	/* R = (H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')*(H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')' */
	float cov_L = 300;
	float cov_l = 300;
	//float cov_h = 0.18;
	float cov_h = 0.3f;
	float cov_vN = 0.1;
	float cov_vE = 0.1;
	//float cov_vD = 0.3f;
	float cov_vD = 0.06f;
	MatZeros(&ekf_t->R);
	ekf_t->R.element[0][0] = cov_L*cov_L;
	ekf_t->R.element[1][1] = cov_l*cov_l;
	ekf_t->R.element[2][2] = cov_h*cov_h;
	ekf_t->R.element[3][3] = cov_vN*cov_vN;
	ekf_t->R.element[4][4] = cov_vE*cov_vE;
	ekf_t->R.element[5][5] = cov_vD*cov_vD;
	
	/* Init P = R */
	MatCopy(&ekf_t->R, &ekf_t->P);
	
	MatZeros(&ekf_t->u);
	MatZeros(&ekf_t->z);
	
//	printf("x ");
//	MatDump(&ekf_t->x);
//	
//	printf("F ");
//	MatDump(&ekf_t->F);
//	
//	printf("H ");
//	MatDump(&ekf_t->H);
//	
//	printf("Q ");
//	MatDump(&ekf_t->Q);
//	
//	printf("R ");
//	MatDump(&ekf_t->R);
//	
//	printf("P ");
//	MatDump(&ekf_t->P);
}

void update_F(EKF_Def* ekf_t)
{
	float rad = Deg2Rad(ekf_t->x.element[0][0]*1e-7);
	
	ekf_t->F.element[1][0] = Tx*1e-7*ekf_t->T*ekf_t->x.element[4][0]*tan(rad)/cos(rad);
	ekf_t->F.element[1][4] = Tx*ekf_t->T/cos(rad);
}

void ekf_update(EKF_Def* ekf_t)
{
	float Ty = Tx/cos(Deg2Rad(ekf_t->x.element[0][0]*1e-7));
	//printf("Tx=%f Ty=%f\n", Tx, Ty);
	/* states: [L l h vn ve vd] */
	
	/* calculate F */
	update_F(ekf_t);
	
//	printf("F ");
//	MatDump(&ekf_t->F);
	
	/* Predict */
	/* x_k|k-1 = f(x_k-1|k-1, u_k-1) */
	ekf_t->pre_x.element[0][0] = ekf_t->x.element[0][0] + ekf_t->x.element[3][0]*Tx*ekf_t->T;
	ekf_t->pre_x.element[1][0] = ekf_t->x.element[1][0] + ekf_t->x.element[4][0]*Ty*ekf_t->T;
	ekf_t->pre_x.element[2][0] = ekf_t->x.element[2][0] - ekf_t->x.element[5][0]*ekf_t->T;
	ekf_t->pre_x.element[3][0] = ekf_t->x.element[3][0] + ekf_t->u.element[3][0]*ekf_t->T;
	ekf_t->pre_x.element[4][0] = ekf_t->x.element[4][0] + ekf_t->u.element[4][0]*ekf_t->T;
	ekf_t->pre_x.element[5][0] = ekf_t->x.element[5][0] + ekf_t->u.element[5][0]*ekf_t->T;
	
//	printf("pre_x ");
//	MatDump(&ekf_t->pre_x);
//	time_waitMs(10);
	
	/* P_k|k-1 = F_k-1*P_k-1|k-1*(F_k-1)' + Q_k-1 */
	MatMul(MatMul(&ekf_t->F, &ekf_t->P, &temp_mat1), MatTrans(&ekf_t->F, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->Q, &ekf_t->pre_P);
//	printf("F ");
//	MatDump(&ekf_t->F);
//	time_waitMs(10);
//	printf("pre P ");
//	MatDump(&ekf_t->pre_P);
//	time_waitMs(10);
	
	/* Update */
	/* y_k = z_k - h(x_k_k-1) */
	MatSub(&ekf_t->z, &ekf_t->pre_x, &ekf_t->y);
//	printf("y ");
//	MatDump(&ekf_t->y);
	/* S_k = H_k*P_k|k-1*(H_k)' + R_k */
	MatMul(MatMul(&ekf_t->H, &ekf_t->pre_P, &temp_mat1), MatTrans(&ekf_t->H, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->R, &ekf_t->S);
//	printf("S ");
//	MatDump(&ekf_t->S);
	/* K_k = P_k|k-1*(H_k)'*(S_k)^-1 */
	MatMul(MatMul(&ekf_t->pre_P, MatTrans(&ekf_t->H, &temp_mat3), &temp_mat1), MatInv(&ekf_t->S, &temp_mat2), &ekf_t->K);
//	printf("K ");
//	MatDump(&ekf_t->K);
//	time_waitMs(10);
	/* x_k|k = x_k|k-1 + K_k*y_k */
	MatAdd(&ekf_t->pre_x, MatMul(&ekf_t->K, &ekf_t->y, &temp_mat4), &ekf_t->x);
//	printf("x ");
//	MatDump(&ekf_t->x);
	/* P_k|k = (I - K_k*H_k)*P_k|k-1 */
	MatEye(&temp_mat1);
	MatMul(MatSub(&temp_mat1, MatMul(&ekf_t->K, &ekf_t->H, &temp_mat2), &temp_mat3), &ekf_t->pre_P, &ekf_t->P);
//	printf("temp1 ");
//	MatDump(&temp_mat1);
//	time_waitMs(10);
//	printf("temp2 ");
//	MatDump(&temp_mat2);
//	time_waitMs(10);
//	printf("temp3 ");
//	MatDump(&temp_mat3);
//	time_waitMs(10);
//	printf("P ");
//	MatDump(&ekf_t->P);
//	
//	printf("\n");

//	time_waitMs(500);
}

void EKF_Init(EKF_Def* ekf_t, float dT)
{
	int state_num = 2;
	MatCreate(&ekf_t->x, state_num, 1);
	MatCreate(&ekf_t->pre_x, state_num, 1);
	MatCreate(&ekf_t->u, state_num, 1);
	MatCreate(&ekf_t->F, state_num, state_num);
	MatCreate(&ekf_t->P, state_num, state_num);
	MatCreate(&ekf_t->pre_P, state_num, state_num);
	MatCreate(&ekf_t->Q, state_num, state_num);
	MatCreate(&ekf_t->H, state_num, state_num);
	MatCreate(&ekf_t->R, state_num, state_num);
	MatCreate(&ekf_t->z, state_num, 1);
	MatCreate(&ekf_t->y, state_num, 1);
	MatCreate(&ekf_t->S, state_num, state_num);
	MatCreate(&ekf_t->K, state_num, state_num);
	
	MatCreate(&temp_mat1, state_num, state_num);
	MatCreate(&temp_mat2, state_num, state_num);
	MatCreate(&temp_mat3, state_num, state_num);
	MatCreate(&temp_mat4, state_num, 1);
	
	ekf_t->T = dT;
	
	/* init F and H */
	MatEye(&ekf_t->F);
	ekf_t->F.element[0][1] = ekf_t->T;
	MatEye(&ekf_t->H);
	
	// sigma_alt = 0.3756	sigma_acc = 0.2350	sigma_v = 0.2	可以用的数据，速度误差比较大
	// sigma_alt = 0.3756	sigma_acc = 0.2350	sigma_pre_v = 0.2 sigma_v = 0.4	比较好的数据
	float sigma_alt = 0.3756;	
	float sigma_acc = 0.2350;
	float sigma_pre_v = 0.2;
	float sigma_v = 0.4;
	
	/* Q = (B*[cov_aN cov_aE cov_aD]') * (B*[cov_aN cov_aE cov_aD]')' */
	MatZeros(&ekf_t->Q);
	ekf_t->Q.element[0][0] = dT*dT*sigma_pre_v*sigma_pre_v;
	ekf_t->Q.element[1][1] = dT*dT*sigma_acc*sigma_acc;
	
	/* R = (H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')*(H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')' */
	MatZeros(&ekf_t->R);
	ekf_t->R.element[0][0] = sigma_alt*sigma_alt;
	ekf_t->R.element[1][1] = sigma_v*sigma_v;
	
	/* Init P = R */
	MatCopy(&ekf_t->R, &ekf_t->P);
	
	MatZeros(&ekf_t->u);
	MatZeros(&ekf_t->z);
}

void EKF_Update(EKF_Def* ekf_t)
{
	/* Predict */
	/* x_k|k-1 = f(x_k-1|k-1, u_k-1) */
	ekf_t->pre_x.element[0][0] = ekf_t->x.element[0][0] + ekf_t->x.element[1][0]*ekf_t->T;
	ekf_t->pre_x.element[1][0] = ekf_t->x.element[1][0] + ekf_t->u.element[1][0]*ekf_t->T;
	/* P_k|k-1 = F_k-1*P_k-1|k-1*(F_k-1)' + Q_k-1 */
	MatMul(MatMul(&ekf_t->F, &ekf_t->P, &temp_mat1), MatTrans(&ekf_t->F, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->Q, &ekf_t->pre_P);

	/* Update */
	/* H is identity Matrix */
	/* y_k = z_k - h(x_k_k-1) */
	MatSub(&ekf_t->z, &ekf_t->pre_x, &ekf_t->y);
	/* S_k = H_k*P_k|k-1*(H_k)' + R_k */
	MatAdd(&ekf_t->pre_P, &ekf_t->R, &ekf_t->S);
	/* K_k = P_k|k-1*(H_k)'*(S_k)^-1 */
	MatMul(&ekf_t->pre_P, MatInv(&ekf_t->S, &temp_mat2), &ekf_t->K);
	/* x_k|k = x_k|k-1 + K_k*y_k */
	MatAdd(&ekf_t->pre_x, MatMul(&ekf_t->K, &ekf_t->y, &temp_mat4), &ekf_t->x);
	/* P_k|k = (I - K_k*H_k)*P_k|k-1 */
	MatSub(&ekf_t->pre_P, MatMul(&ekf_t->K, &ekf_t->pre_P, &temp_mat1), &ekf_t->P);
}


