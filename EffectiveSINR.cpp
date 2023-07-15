//EffectiveSINR.cpp: 主项目文件。
#include "EffectiveSINR.h"
#include"wifi-bandwidth-manager.h"
#include"lte-bandwidth-manager.h"
#include <iostream>
#include <math.h>
#include <cstddef>
#include <vector>
#include <stdexcept>
using namespace std;
double EffectiveSINR::WIFI_SINRTableBegin[10] = { -9,-3, 1, 2, 4, 8, 9, 14, 19, 20 };
double EffectiveSINR::WIFI_beta[80]={2.46,2.3,2.27,2.18,2.05,2.00,2.03,2.04,1.98,2.56,2.43,2.46,2.41,2.41,2.38,7.45,7.14,7.00,7.34,6.89,8.93,8.87,8.85,11.31,11.11,11.09,13.80,13.69, 4.71};
double EffectiveSINR::LTE_SINRTableBegin[15] = { -14.5, -12.5, -10.5, -8.5, -6.5, -4.5, -4.5, -0.5, 1.5, 3.5, 5.5, 7.0, 8.5, 10.25, 12 };
double EffectiveSINR::LTE_beta[29] = { 2.46, 2.28, 2.27, 2.18, 2.05, 2.00, 2.03, 2.04, 1.98, 2.56, 2.43, 2.46, 2.41, 2.41, 2.38, 7.45, 7.14, 7.00, 7.34, 6.89, 8.93, 8.87, 8.85, 11.31, 11.11, 11.09, 13.80, 13.69, 14.71 };

double 
EffectiveSINR::WIFI_eesmSINReff(double ptr[], int MCS, int Nss)  //载频SINR、mcs[0-9]、空间流数        目前NSS=1
{
	double b = WIFI_beta[MCS + (Nss - 1) * 10];    //
	double temp = 0;
	for (int i = 0; i <RBs_FOR_WIFI; i++)
	{
		if (ptr[i]!=0)
         temp += exp(-(ptr[i]) / b);      //等效信干燥比
	}
	return  10*log10(-b*log(temp / RBs_FOR_WIFI));
};

double
EffectiveSINR::LTE_eesmSINReff(double ptr[], int MCS, int Nss)  //载频SINR、mcs[0-29]、空间流数        目前NSS=1
{
	double b = LTE_beta[MCS + (Nss - 1) * 10];
	double temp = 0;
	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		if (ptr[i] != 0)
		temp += exp(-(ptr[i]) / b);      //等效信干燥比
	}
	return  10*log10(-b*log(temp / RBs_FOR_LTE));
}



