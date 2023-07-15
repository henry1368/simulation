#ifndef _RANDOM_H_
#define _RANDOM_H_
#include <iostream>
#include <list>
#include <algorithm>
#include <string>
using namespace std;
/*
*定义Random类，用于产生各种随机数
*/
class Random {
public:
	//构造函数
	Random();
	//析构函数
	~Random();
	//初始化种子
	void ReInit();									// Change the seed of random number generation
	//指数分布函数
	double Exponential(double lamda);
	//帕累托分布函数
	double Pareto(double a, double k,double max);
	//对数正太分布函数
	double Normal(double Mu, double Theta);
	double LogN(double miu, double sigma);

	//产生均匀分布函数
	double AverageRandom(double min, double max);
	//极大值分布

	double FisherTippettRan(double a, double b);
	////
};



#endif