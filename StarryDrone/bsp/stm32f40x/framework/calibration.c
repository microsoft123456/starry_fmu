/**
* File      : calibration.c
*
* 最小二乘法椭球拟合校正算法
*
* Change Logs:
* Date      	Author       	Notes
* 2016-06-30	zoujiachi		the first version
*/

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>
#include <math.h>

#define MATRIX_SIZE 7
#define u8 unsigned char

double m_matrix[MATRIX_SIZE][MATRIX_SIZE+1];
int m = MATRIX_SIZE;	
int n = MATRIX_SIZE+1;
double m_result[MATRIX_SIZE];	

void DispMatrix(void);

double Abs(double a)
{
	return a<0 ? -a : a;
}

u8 Equal(double a,double b)
{
	return Abs(a-b) < 1e-8;
}

void ResetMatrix(void)
{
	int row , column;
	
	for(row = 0 ; row<m ; row++){
		for(column = 0 ; column<n ; column++)
			m_matrix[row][column] = 0.0f;
	}
}
	
void CalcData_Input(double x , double y , double z)
{
	double V[MATRIX_SIZE];
	int row , column;
	
	V[0] = x*x;
    V[1] = y*y;
    V[2] = z*z;
    V[3] = x;
    V[4] = y;
    V[5] = z;
    V[6] = 1.0;
	
	//构建VxVt矩阵(Vt为V的转置)，并进行累加
	for(row = 0 ; row<MATRIX_SIZE ; row++){
		for(column = 0 ; column<MATRIX_SIZE ; column++){
			m_matrix[row][column] += V[row]*V[column];
		}
	}
}

void SwapRow(int row1 , int row2)
{
	int column;
	double tmp;
	
	for(column = 0 ; column<n ; column++){
		tmp = m_matrix[row1][column];
		m_matrix[row1][column] = m_matrix[row2][column];
		m_matrix[row2][column] = tmp;
	}
}

void MoveBiggestElement2Top(int s_row , int s_column)
{
	int row;
	
	for(row = s_row+1 ; row<m ; row++){
		if( Abs(m_matrix[s_row][s_column])<Abs(m_matrix[row][s_column])){
			SwapRow(s_row , row);
		}
	}
}

//高斯消元法，求行阶梯型矩阵
u8 Matrix_GaussElimination(void)
{
	int row,column,i,j;
	double tmp;
	
	for(row = 0,column=0 ; row<m-1 && column<n-1 ; row++,column++){
		//将当前列最大的一行移上来
		MoveBiggestElement2Top(row , column);
		
		//整列都为0
		if(Equal(m_matrix[row][column],0.0f)){
			//printf("qiyi matrix:%d %d\r\n" , row , column);
			//DispMatrix();
			//return 0;
			row--;
			continue;
		}
		
		//高斯消元
		for(i = row+1 ; i<m ; i++){	
			if(Equal(m_matrix[i][column],0.0f))
				continue;	//为0，无需处理
			
			tmp = m_matrix[i][column]/m_matrix[row][column];
			
			for(j = column ; j<n ; j++){
				m_matrix[i][j] -= m_matrix[row][j]*tmp;
			}
		}

//		DispMatrix();
//		printf("\r\n");
	}

	return 1;
}

//求行最简型矩阵
int Matrix_RowSimplify(void)
{
    int c = n;//返回值，表示(解的任意常量数+1)；
    //
    int row,column,s,t;
    //
    for(row=0,column=0;row<m && column<n;row++,column++)
    {
        if(Equal(m_matrix[row][column],0))//平移，找出本行第一个非零；
        {
            row--;
            continue;
        }
        //
        c--;
        //
		//这里不化成对角矩阵为1的矩阵，为了防止输入的数据较大的时候，求出的解为接近于0值的情况
        //tmp = 1 / m_matrix[row][column];
        //for(k=column;k<n;k++)//前面的"0"就不处理了；
            //m_matrix[row][k] *= tmp;
        //
        //化上三角矩阵为对角矩阵
		
		if(row == m-1)
			m_matrix[row][column] = 0.0f;	//强制为0，释放一个自由度，否则很难有解
		
        for(s=0;s<row;s++)//下面的0也不用处理；
        {
			float tmp;
			
            if(Equal(m_matrix[s][column],0))
                continue;//已经为0；
            //
            tmp = m_matrix[s][column]/m_matrix[row][column];
            for(t=column;t<n;t++)
                m_matrix[s][t] -= m_matrix[row][t]*tmp;
            //
        }
//		DispMatrix();
//		printf("\r\n");
    }
    //
    return c;
}

void Matrix_Solve(double* C , double* sol)
{
	int row,column,i;
	int any_sol[MATRIX_SIZE];

	//找出任意解的位置
	memset(any_sol , 0 , MATRIX_SIZE);
	for(row=0,column=0 ; row<m && column<n-1 ; row++,column++){
		if(Equal(m_matrix[row][column] , 0.0f)){
			any_sol[column] = 1;	//记录任意解的位置
			row--;	//右移1列
		}
	}

	//求解
	row = 0;
	for(column = 0 ; column<n-1 ; column++){
		if(any_sol[column] == 1){	//任意解
			sol[column] = C[column];
		}else{
			sol[column] = m_matrix[row][n-1];
			//加上任意解
			for(i = column+1 ; i<n-1 ; i++){
				if(any_sol[i]==1 && !Equal(m_matrix[row][i],0.0f)){
					sol[column] -= m_matrix[row][i]*C[i];
				}
			}	
			sol[column] /= m_matrix[row][column];        //除以对角线元素
			row++;
		}
	}
}

void DispMatrix(void)
{
	int row,column;
	
	for(row = 0 ; row<m ; row++){
		for(column = 0 ; column<n ; column++){
			printf("%.2f		" , m_matrix[row][column]);
		}
		printf("\r\n");
	}
}

double* calibrate_process(double radius)
{
	double C[MATRIX_SIZE];
	double Res[MATRIX_SIZE];
	int i;
	double k;
	
	Matrix_GaussElimination();
	Matrix_RowSimplify();

    //赋值任意解参数值
	for(i = 0 ; i<MATRIX_SIZE ; i++){
		C[i] = 1000.0f;
	}

	Matrix_Solve(C , Res);

	printf("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n" , Res[0],Res[1],Res[2],Res[3],Res[4],Res[5],Res[6]);
	
	k = (Res[3]*Res[3]/Res[0]+Res[4]*Res[4]/Res[1]+Res[5]*Res[5]/Res[2] - 4*Res[6])/(4*radius*radius);

	if(Res[0]*k<0 || Res[1]*k<0 || Res[2]*k<0)
	{
		printf("data is not typical\r\n");
		return RT_NULL;
	}
	
	m_result[0] = sqrt(Res[0] / k);
    m_result[1] = sqrt(Res[1] / k);
    m_result[2] = sqrt(Res[2] / k);
    m_result[3] = Res[3] / (2 * Res[0]);
    m_result[4] = Res[4] / (2 * Res[1]);
    m_result[5] = Res[5] / (2 * Res[2]);

	printf("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f k:%f\r\n" , m_result[3],m_result[4],m_result[5],m_result[0],m_result[1],m_result[2],k);

	return m_result;
}

void cali_input_acc_data(uint16_t p_num)
{
	uint16_t i;
	float acc[3];
	
	for(i = 0 ; i<p_num ; i++)
	{
		sensor_acc_measure(acc);
		printf("(%.2f,%.2f,%.2f) " , acc[0],acc[1],acc[2]);
		CalcData_Input(acc[0], acc[1], acc[2]);
		time_waitMs(20);
	}
	rt_kprintf("\r\n");
}

void cali_input_mag_data(uint16_t p_num)
{
	uint16_t i;
	float mag[3];
	
	for(i = 0 ; i<p_num ; i++)
	{
		sensor_mag_measure(mag);
		printf("(%.2f,%.2f,%.2f) " , mag[0],mag[1],mag[2]);
		CalcData_Input(mag[0], mag[1], mag[2]);
		time_waitMs(20);
	}
	rt_kprintf("\r\n");
}

rt_err_t calibrate_gyr(uint16_t p_num)
{
	float* gyr_data_p;
	float sum_gyr[3] = {0.0f,0.0f,0.0f};
	float offset_gyr[3];
	
	gyr_data_p = rt_malloc(p_num*12);
	
	if(gyr_data_p == RT_NULL)
		return RT_ERROR;
	
	for(uint16_t i = 0 ; i<p_num ; i++)
	{
		sensor_gyr_measure(&gyr_data_p[i*3]);
		sum_gyr[0] += gyr_data_p[i*3];
		sum_gyr[1] += gyr_data_p[i*3+1];
		sum_gyr[2] += gyr_data_p[i*3+2];
		printf("(%.2f,%.2f,%.2f) " , gyr_data_p[i*3],gyr_data_p[i*3+1],gyr_data_p[i*3+2]);
		time_waitMs(3);
	}
	rt_kprintf("\r\n");
	
	offset_gyr[0] = -sum_gyr[0]/p_num;
	offset_gyr[1] = -sum_gyr[1]/p_num;
	offset_gyr[2] = -sum_gyr[2]/p_num;
	
	printf("gyr offset:%f %f %f\r\n" , offset_gyr[0],offset_gyr[1],offset_gyr[2]);
	
	return RT_EOK;
}

void Calc_Process(double radius)
{
	double C[MATRIX_SIZE];
	double Res[MATRIX_SIZE];
	int i;
	double k;

	ResetMatrix();

	//输入任意个数磁场测量点坐标，请尽量保证在椭球上分布均匀
//	CalcData_Input(-9.81,-0.20,-0.91);
//	CalcData_Input(-9.71,-0.06,-1.04);
//	CalcData_Input(-9.88,-0.06,-1.14);
//	CalcData_Input(-9.74,-0.17,-0.93);
//	CalcData_Input(-9.76,-0.27,-1.10);
//	CalcData_Input(-9.91,-0.13,-0.95);

//	CalcData_Input(0.19,-9.77,-1.54);
//	CalcData_Input(0.17,-9.72,-1.68);
//	CalcData_Input(0.06,-9.79,-1.73);
//	CalcData_Input(0.18,-9.81,-1.75);
//	CalcData_Input(0.08,-9.75,-1.73);
//	CalcData_Input(0.19,-9.76,-1.68);

//	CalcData_Input(9.94,-0.04,-0.74);
//	CalcData_Input(9.92,-0.04,-0.83);
//	CalcData_Input(10.30,-0.04,-0.96);
//	CalcData_Input(10.07,-0.05,-0.83);
//	CalcData_Input(10.02,-0.10,-0.54);
//	CalcData_Input(9.94,-0.20,-0.52);

//	CalcData_Input(0.22,9.54,-0.66);
//	CalcData_Input(0.01,9.45,-1.05);
//	CalcData_Input(0.18,9.57,-0.73);
//	CalcData_Input(0.11,9.52,-0.98);
//	CalcData_Input(0.24,9.65,-1.18);
//	CalcData_Input(0.10,9.58,-0.95);

//	CalcData_Input(-0.03,-0.17,8.10);
//	CalcData_Input(0.03,-0.08,8.29);
//	CalcData_Input(0.06,-0.11,8.22);
//	CalcData_Input(0.05,-0.18,8.47);
//	CalcData_Input(-0.03,-0.04,8.23);
//	CalcData_Input(-0.08,-0.21,8.51);

//	CalcData_Input(0.15,-0.07,-10.54);
//	CalcData_Input(0.07,0.19,-10.87); 
//	CalcData_Input(0.33,0.21,-10.93); 
//	CalcData_Input(0.51,0.25,-10.85); 
//	CalcData_Input(-0.17,-0.03,-10.71); 
//	CalcData_Input(-0.19,0.15,-10.77); 

	Matrix_GaussElimination();
	Matrix_RowSimplify();

    //赋值任意解参数值
	for(i = 0 ; i<MATRIX_SIZE ; i++){
		C[i] = 1000.0f;
	}

	Matrix_Solve(C , Res);

	printf("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n" , Res[0],Res[1],Res[2],Res[3],Res[4],Res[5],Res[6]);

	k = (Res[3]*Res[3]/Res[0]+Res[4]*Res[4]/Res[1]+Res[5]*Res[5]/Res[2] - 4*Res[6])/(4*radius*radius);

	m_result[0] = sqrt(Res[0] / k);
    m_result[1] = sqrt(Res[1] / k);
    m_result[2] = sqrt(Res[2] / k);
    m_result[3] = Res[3] / (2 * Res[0]);
    m_result[4] = Res[4] / (2 * Res[1]);
    m_result[5] = Res[5] / (2 * Res[2]);

	printf("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f k:%f\r\n" , m_result[3],m_result[4],m_result[5],m_result[0],m_result[1],m_result[2],k);
}
