// bler_curves.cpp: ����Ŀ�ļ���
#include "BLERCurves.h"
#include <math.h>
#include <stdlib.h> 
#include <time.h> 

double BLERCurves::resolution = 0.25;//��������ߵĲ���Ϊ0.25

double BLERCurves::WIFI_SINRTableBegin[10] = { -9, -3, 1, 2, 4, 8, 9, 14, 19, 20 };
double BLERCurves::LTE_SINRTableBegin[15] = { -14.5, -12.5, -10.5, -8.5, -6.5, -4.5, -4.5, -0.5, 1.5, 3.5, 5.5, 7.0, 8.5, 10.25, 12 };

double BLERCurves::WIFI_blerTable[10][43] = {  
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9970, 0.9990, 0.9940, 0.9920, 0.9890, 0.9730, 0.9490, 0.9220, 0.8710, 0.8550, 0.7410, 0.6610, 0.5680, 0.4910, 0.3900, 0.3180, 0.2410, 0.1920, 0.1010, 0.0740, 0.0450, 0.0250, 0.0250, 0.0080, 0.0020, 0.0020, 0.0020, 0.0020, 0.0010, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 0.9990, 0.9960, 0.9810, 0.9740, 0.9270, 0.9090, 0.8100, 0.6930, 0.6510, 0.5310, 0.4480, 0.3530, 0.2660, 0.2100, 0.1710, 0.1530, 0.1190, 0.0940, 0.0640, 0.0690, 0.0640, 0.0400, 0.0330, 0.0280, 0.0320, 0.0160, 0.0140, 0.0190, 0.0110, 0.0070, 0.0060, 0.0020, 0.0030, 0.0040, 0.0020, 0.0010, 0.0030, 0, 0 },
	{ 1.0000, 0.9950, 0.9880, 0.9720, 0.9420, 0.8810, 0.8040, 0.7300, 0.6430, 0.5170, 0.4160, 0.2660, 0.2110, 0.1510, 0.1110, 0.0620, 0.0640, 0.0350, 0.0310, 0.0290, 0.0160, 0.0110, 0.0080, 0.0070, 0.0030, 0.0020, 0.0030, 0, 0, 0, 0.0010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9980, 0.9940, 0.9830, 0.9690, 0.9280, 0.8980, 0.8040, 0.7020, 0.5930, 0.5130, 0.4270, 0.2950, 0.2490, 0.1990, 0.1640, 0.1050, 0.1030, 0.0790, 0.0650, 0.0380, 0.0400, 0.0340, 0.0320, 0.0260, 0.0210, 0.0070, 0.0065, 0.0060, 0.0050, 0.0040, 0.0030, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9980, 0.9950, 0.9900, 0.9710, 0.9160, 0.8680, 0.7730, 0.6590, 0.5820, 0.4560, 0.3580, 0.2510, 0.1970, 0.1430, 0.1010, 0.0570, 0.0560, 0.0290, 0.0160, 0.0100, 0.0070, 0.0040, 0.0020, 0.0020, 0, 0, 0.0010 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9970, 0.9920, 0.9840, 0.9600, 0.9330, 0.8580, 0.7650, 0.6690, 0.5210, 0.4320, 0.3170, 0.2340, 0.1720, 0.1110, 0.0880, 0.0420, 0.0240, 0.0150, 0.0100, 0.0060, 0.0030, 0.0010, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9990, 0.9960, 0.9930, 0.9710, 0.9590, 0.9080, 0.8320, 0.7390, 0.6140, 0.5280, 0.4110, 0.3050, 0.2410, 0.1620, 0.1100, 0.0850, 0.0520, 0.0400, 0.0230, 0.0140, 0.0060, 0.0040, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 0.9990, 0.9960, 0.9820, 0.9610, 0.9240, 0.8700, 0.7790, 0.6800, 0.5640, 0.4300, 0.3430, 0.2510, 0.1670, 0.1260, 0.0720, 0.0410, 0.0370, 0.0160, 0.0170, 0.0130, 0.0030, 0.0030, 0, 0.0010, 0.0010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9990, 0.9870, 0.9730, 0.9520, 0.9050, 0.8110, 0.7290, 0.6410, 0.5200, 0.3880, 0.3020, 0.2330, 0.1570, 0.0960, 0.0710, 0.0510, 0.0460, 0.0310, 0.0240, 0.0170, 0.0170, 0.0100, 0.0060, 0.0010, 0, 0.0050, 0.0040, 0, 0.0010, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9950, 0.9820, 0.9580, 0.9240, 0.8600, 0.7520, 0.6340, 0.5140, 0.3740, 0.3200, 0.2160, 0.1440, 0.0840, 0.0700, 0.0350, 0.0370, 0.0070, 0.0110, 0.0070, 0.0070, 0.0030, 0.0030, 0, 0.0010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

};

double BLERCurves::LTE_blerTable[15][43] = {
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9998, 0.9998, 0.9998, 0.9988, 0.9984, 0.9956, 0.9862, 0.9732, 0.9384, 0.8938, 0.8144, 0.7088, 0.5742, 0.4392, 0.2864, 0.1818, 0.0988, 0.0476, 0.0192, 0.0086, 0.0022, 0.0004, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9998, 1.0000, 0.9998, 0.9990, 0.9964, 0.9906, 0.9724, 0.9188, 0.8558, 0.7332, 0.5684, 0.3820, 0.2232, 0.1092, 0.0486, 0.0124, 0.0048, 0.0018, 0.0002, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9996, 0.9996, 0.9968, 0.9834, 0.9460, 0.8518, 0.6990, 0.4770, 0.2664, 0.1112, 0.0410, 0.0092, 0.0012, 0, 0.0002, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9998, 0.9980, 0.9870, 0.9296, 0.7790, 0.5356, 0.2718, 0.0902, 0.0192, 0.0032, 0.0010, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9998, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9956, 0.9736, 0.8534, 0.5952, 0.2762, 0.0702, 0.0110, 0.0018, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9996, 0.9958, 0.9694, 0.8184, 0.4972, 0.1760, 0.0300, 0.0026, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9988, 0.9842, 0.8918, 0.5888, 0.2236, 0.0402, 0.0028, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9998, 0.9996, 0.9958, 0.9434, 0.6908, 0.2936, 0.0522, 0.0032, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9926, 0.9452, 0.7416, 0.3608, 0.0642, 0.0044, 0.0008, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9996, 0.9842, 0.8498, 0.4582, 0.1058, 0.0084, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9994, 0.9740, 0.7564, 0.3054, 0.0488, 0.0034, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9906, 0.9052, 0.5670, 0.1548, 0.0150, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9950, 0.8998, 0.5164, 0.1344, 0.0180, 0.0022, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9996, 0.9510, 0.6506, 0.1864, 0.0160, 0.0012, 0.0008, 0.0002, 0.0002, 0.0002, 0, 0.0002, 0, 0, 0, 0 },
	{ 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9968, 0.9550, 0.6856, 0.2942, 0.0720, 0.0166, 0.0034, 0.0010, 0.0002, 0, 0.0002, 0, 0, 0, 0 }
};

double BLERCurves::WIFI_getBLER(double SINReff, int MCS, int Nss)
{
	int index = (MCS + Nss - 1) ;


	int i;
	double p;
	p = WIFI_SINRTableBegin[index];
	if (SINReff<p)                //SINR������ 
		return 1;
	else if (SINReff>p+10.5)       //SINR������
		return 0;
	else
	{
		i = (int)ceil((SINReff - p) / resolution);
		
		return WIFI_blerTable[index][i] + (p + i*resolution - SINReff)*(WIFI_blerTable[index][i - 1] - WIFI_blerTable[index][i]) / resolution;
	}
};

double BLERCurves::LTE_getBLER(double SINReff, int MCS) 
{
	int i;
	double p, temp;

	p = LTE_SINRTableBegin[MCS - 1];

	if (SINReff < p)                //�����о�����֤�������ָ������ 
		return 1;
	else if (SINReff > p + 10.5)
		return 0;
	else
	{
		temp = (SINReff - p) / resolution;

		i = (int)ceil(temp);
		return LTE_blerTable[MCS - 1][i] - (i - temp)*(LTE_blerTable[MCS - 1][i] - LTE_blerTable[MCS - 1][i - 1]);
	}
};