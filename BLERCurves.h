#ifndef BLERCurves_H
#define BLERCurves_H

class BLERCurves
{
public:
	BLERCurves(){};

	static double WIFI_getBLER(double SINReff, int MCS, int Nss);
	static double WIFI_blerTable[10][43];
	static double WIFI_SINRTableBegin[10];

	static double LTE_getBLER(double SINReff, int MCS);
	static double LTE_blerTable[15][43];
	static double LTE_SINRTableBegin[15];

	static double resolution;
};
#endif
