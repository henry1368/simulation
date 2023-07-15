#ifndef FFTMATH_H
#define FFTMATH_H

#include <complex>
#include <iostream>
#include"parameter.h"
using namespace std;

//const double PI = 3.141592;			//PI
//const double light_speed = 3e8;		//光速




 static double gaussrand()
{
	static double V1 = 0, V2 = 0, S = 0;
	static int phase = 0;
	double X = 0;
	double U1, U2;
	if (phase == 0)
	{

		U1 = (double)rand();
		U2 = (double)rand();

		U1 = U1 / RAND_MAX;
		U2 = U2 / RAND_MAX;

		V1 = 2 * U1 - 1;
		V2 = 2 * U2 - 1;
		S = V1*V1 + V2*V2;

		if (S >= 1)	S = S - 0.9999;
		if (S >= 1)	S = S - 0.0002;
		if (S <= 0) S = 0.0001;

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}


//定义旋转因子
 static complex<double> W(int N, int n)
{
	complex<double> out((double)cos(2 * PI*((double)n / (double)N)), -(double)sin(2 * PI*((double)n / (double)N)));
	return out;
}


 static complex<double>* DOFFT(complex<double> *x, complex<double> *F, int N)
{

	int K = (int)(log((double)N + 1) / log((double)2));

	for (int m = 1; m <= K; m++)
	{
		for (int i = 0; i < (1 << K); i += 1 << m)
		{
			for (int j = 0; j < (1 << (m - 1)); j++)
			{
				F[i + j] = x[i + j] + x[i + j + (1 << (m - 1))] * W(1 << m, j);
				F[i + j + (1 << (m - 1))] = x[i + j] - x[i + j + (1 << (m - 1))] * W(1 << m, j);
			}
		}

		for (int i = 0; i < N; i++)
			x[i] = F[i];
	}

	return F;
}

 static int reverse(int n, int j)
{
	int  i, k, power;
	power = (int)(log((double)n + 1) / log((double)2));
	k = 0;
	for (i = 0; i < power; i++)//检查每个为0~power-1
	if (j&(1 << i))
		k += 1 << (power - i - 1);//如果正序数中位i的值为1，则倒序数中power-i-1的相应位置1。
	return k;
}


 static  void FFTCalculation(complex<double> *time_data, complex<double> *freq_data, int fft_len)
{
	int l, i, j;
	int N = fft_len;
	complex<double> *tmp_data_left = new complex<double>[fft_len];
	complex<double> *tmp_data_right = new complex<double>[fft_len];
	for (l = 0; l < fft_len; l++)
	{
		tmp_data_left[l] = time_data[l];
	}

	for (l = 0; l < fft_len; l++)
	{
		tmp_data_right[l] = complex<double>(0, 0);
	}

	for (i = 0; i < N; i++)
	{
		tmp_data_right[i] = tmp_data_left[reverse(N, i)];
	}
	for (i = 0; i < N; i++)
	{
		tmp_data_left[i] = tmp_data_right[i];
	}

	DOFFT(tmp_data_left, tmp_data_right, N);

	for (j = 0; j < fft_len; j++)
	{
		freq_data[j] = tmp_data_right[j];//	freq_data[j] = tmp_data_right[j] / sqrt((double)fft_len);
	}

	delete[] tmp_data_left;
	delete[] tmp_data_right;
}

#endif
