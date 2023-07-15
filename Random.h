#ifndef _RANDOM_H_
#define _RANDOM_H_
#include <iostream>
#include <list>
#include <algorithm>
#include <string>
using namespace std;
/*
*����Random�࣬���ڲ������������
*/
class Random {
public:
	//���캯��
	Random();
	//��������
	~Random();
	//��ʼ������
	void ReInit();									// Change the seed of random number generation
	//ָ���ֲ�����
	double Exponential(double lamda);
	//�����зֲ�����
	double Pareto(double a, double k,double max);
	//������̫�ֲ�����
	double Normal(double Mu, double Theta);
	double LogN(double miu, double sigma);

	//�������ȷֲ�����
	double AverageRandom(double min, double max);
	//����ֵ�ֲ�

	double FisherTippettRan(double a, double b);
	////
};



#endif