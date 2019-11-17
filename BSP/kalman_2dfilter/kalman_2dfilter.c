#include "kalman_2dfilter.h"

#define T  1
#define M  5
float P[4][4] = { {1, 0,0,0 },{ 0, 1,0,0 },{ 0, 0,1,0 },{0, 0,0,1}};
float X[4][1] = {{0 }, {0 } ,{ 0 } ,{ 0 } };
float F[4][4] = {{1, T ,0,0}, {0, 1,0,0 },{ 0,0,1,T },{ 0,0,0,1 } };
float FT[4][4] = { {1, 0 ,0,0}, {T, 1 ,0,0},{0,0,1,0 } ,{0,0,T,1}};//
float Q[4][4] = { { 0.001, 0,0,0 },{ 0, 0.001,0,0 },{ 0, 0,0.001,0 } ,{ 0,0,0,0.001 } };
int H[2][4] = { {1 , 0 ,0, 0},{ 0 , 0 ,1, 0 } };
int HT[4][2] = { { 1,0 } ,{ 0,0 },{ 0,1 },{ 0,0 } };
int R[2][2] = { { 1, 0},{ 0, 1}};
float X_[4][1];
float P_[4][4];
float K[4][2];
float* KM(float S[2][1]);

float* mmm;// = 0.0;//[4][1] = { { 0 },{ 0 } ,{ 0 } ,{ 0 } };

float* KM(float S[2][1])
{
	float matrix[M][M];
	float matrix1[M][M];

	int i,j,k;
	//状态转移
	X_[0][0] = X[0][0] + X[1][0] * T;
	X_[1][0] = X[1][0];
	X_[2][0] = X[2][0] + X[3][0] * T;
	X_[3][0] = X[3][0];
	//状态转移
	
	//协方差转移
	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix[i][j] = matrix[i][j] + F[i][k] * P[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix1[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix1[i][j] = matrix1[i][j] + matrix[i][k] * FT[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			P_[i][j] = matrix1[i][j] + Q[i][j];
		}
	}
	//协方差转移
	//计算K
	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<2;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix[i][j] = matrix[i][j] + P[i][k] * HT[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix1[i][j] = 0;
		}
	}

	for (i = 0;i<2;i++)
	{
		for (j = 0;j<2;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix1[i][j] = matrix1[i][j] + H[i][k] * matrix[k][j];
			}
		}
	}

	for (i = 0;i<2;i++)
	{
		for (j = 0;j<2;j++)
		{
			matrix1[i][j] = matrix1[i][j] + R[i][j];
		}
	}
	float temp = 1 / (matrix1[1][1] * matrix1[0][0] - matrix1[0][1] * matrix1[1][0]);

	//求二阶矩阵的逆
	matrix1[0][0] = temp * matrix1[1][1];
	matrix1[0][1] = -1* temp * matrix1[0][1];
	matrix1[1][0] = -1 * temp * matrix1[1][0];
	matrix1[1][1] = temp * matrix1[0][0];
	//求二阶矩阵的逆

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<2;j++) {
			K[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<2;j++)
		{
			for (k = 0;k<2;k++)
			{
				K[i][j] = K[i][j] + matrix[i][k] * matrix1[k][j];
			}
		}
	}

	//计算K

	//X
	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix[i][j] = 0;
		}
	}

	for (i = 0;i<2;i++)
	{
		for (j = 0;j<1;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix[i][j] = matrix[i][j] + H[i][k] * X_[k][j];
			}
		}
	}

	for (i = 0;i<2;i++)
	{
		for (j = 0;j<1;j++)
		{
			matrix[i][j] = S[i][j] - matrix[i][j];
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix1[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<1;j++)
		{
			for (k = 0;k<2;k++)
			{
				matrix1[i][j] = matrix1[i][j] + K[i][k] * matrix[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<1;j++)
		{
			X[i][j] = X_[i][j] + matrix1[i][j];
		}
	}
	//X

	//P
	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix[i][j] = 0;
		}
	}


	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			for (k = 0;k<2;k++)
			{
				matrix[i][j] = matrix[i][j] + K[i][k] * H[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			matrix1[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			for (k = 0;k<4;k++)
			{
				matrix1[i][j] = matrix1[i][j] + matrix[i][k] * P_[k][j];
			}
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++) {
			P[i][j] = 0;
		}
	}

	for (i = 0;i<4;i++)
	{
		for (j = 0;j<4;j++)
		{
			P[i][j] = P_[i][j] - matrix1[i][j];
		}
	}
	return *X;//, X[2][0];
}

