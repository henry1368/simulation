#ifndef EffectiveSINR_H
#define EffectiveSINR_H

#include "parameter.h"

using namespace std;
class EffectiveSINR
{
public:
	EffectiveSINR(){};
	static double WIFI_eesmSINReff(double ptr[], int MCS, int Nss);    //EESM       ptr为载频点SINR的一维数组
	static double WIFI_SINRTableBegin[10];
	static double WIFI_beta[80];

	static double LTE_eesmSINReff(double ptr[], int MCS, int Nss);
	static double LTE_SINRTableBegin[15];
	static double LTE_beta[29];
};
#endif
