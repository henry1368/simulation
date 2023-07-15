#include "Random.h"
#include<stdlib.h>
#include<time.h>
#include<ctime>
#include<math.h>



//���캯��

Random::Random()
{
	ReInit();
}
//��������
Random::~Random()
{
}

void Random::ReInit() {

	srand((unsigned int)time((time_t *)NULL));

}

double Random::Exponential(double lamda)
{//��������ָ���ֲ��������
	double f;
	//	srand((unsigned int)time((time_t *)NULL));
	f = ((double)rand() + 1) / (RAND_MAX + 1);
	return -log(1 - f) / lamda;
}


double Random::Pareto(double a, double k,double max)
{//�����������зֲ��������
	double f;
	//	srand((unsigned int)time((time_t *)NULL));
	f = (double)rand() / RAND_MAX;
	f = pow(f, 1 / a);
	f = k / f;
	if (f > max)
	{
		f = max;
	}
	return f;
}

//�������Ӷ�����̬�ֲ��������
double Random::Normal(double Mu, double Theta)
{
	double u, t;
	int i;
	int n = 5000;
	u = (double)rand() / RAND_MAX;
	for (i = 1; i<n; i++)
	{
		u = u + (double)rand() / RAND_MAX;
	}
	double rr = n / 12;
	t = (u - n / 2) / sqrt(rr);
	return (Mu + Theta * t);
}

double Random::LogN(double miu, double sigma)
{
	double x;
	double y;
	x = Normal(miu, sigma);
	y = exp(x);
	return(y);
}





double Random::FisherTippettRan(double a, double b)
{
	double x;
	double y;
	x = ((double)rand() + 1) / (RAND_MAX + 1);
	y = a - b*log(-log(x));
	return y;

}




