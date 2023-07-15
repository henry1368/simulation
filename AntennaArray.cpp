#include <iostream>
#include <algorithm>
#include "AntennaArray.h"
#include "string.h"
#include "parameter.h"
using namespace std;

AntennaArray::AntennaArray(){};
AntennaArray::~AntennaArray(){};

AntennaArray::AntennaArray(enumAntennaType type, double broadsideAzimuth, char * str)  //Antennatype��ͬ�ĳ����µ�����Ǻͺ����߸߶Ȳ�һ��
{
	dBiGain = 17.;      //dBi
	theta3dB = 70.;     //degree
	phi3dB = 15.;       //degree
	maxA = 20.;         //dB
	//	Ut_height = 1.5;

	if (strcmp(str, "UMi"))
	{
		tiltAngle = 12.;          //degree
			 //  Bs_height = 10;
	}

	if (strcmp(str, "UMa"))
	{
		tiltAngle = 12.;          //degree
		//	  Bs_height = 25;
	}

	if (strcmp(str, "RMa"))
	{
		tiltAngle = 6.;           //degree
		//	  Bs_height = 35;
	}

	if (strcmp(str, "SMa"))
	{
		tiltAngle = 6.;          //degree
	//	    Bs_height = 35;
	}

	if (strcmp(str, "TC1"))
	{
		tiltAngle = 12.;        //degree      //��ʱ����UMi������
	//		 Bs_height = 10;
	}

	if (strcmp(str, "TC2"))
	{
		tiltAngle = 12.;        //degree      //��ʱ����UMi������
		//		 Bs_height = 10;
	}

	antennaType = type;
	broadsideAngle = broadsideAzimuth;
	slantAngle.clear();
	distanceToFirstElement.clear();
	if (antennaType == Antenna_IMTA_bs)
	{
		for (int i = 0; i < Nt; i++)
		{
			distanceToFirstElement.push_back(10.0*i);
			slantAngle.push_back(90.0);
		}
	}
	else if (antennaType == Antenna_IMTA_ms)
	{
		for (int i = 0; i < Nr; i++)
		{
			distanceToFirstElement.push_back(0.5*i);
			slantAngle.push_back(90.0);
		}
	}
}




double AntennaArray::getAzimuthRelativeToBroadside(double absoluteAzimuth)
{
	double relativeAngle = absoluteAzimuth - broadsideAngle;
	return wrapAngleDistribution(relativeAngle);
}

double
AntennaArray::GetAzimuthGain(double azimuth)
{
	double angle = getAzimuthRelativeToBroadside(azimuth);
	double gain = -min(12. * (angle / theta3dB)*(angle / theta3dB), maxA);
	return gain;
}


double
AntennaArray::GetElevationGain(double elevation)
{
	double gain = -min(12.0*((elevation - tiltAngle) / phi3dB)*((elevation - tiltAngle) / phi3dB), maxA);
	return gain;
}


double
AntennaArray::GetGain(double azimuth, double elevation)
{
	double gain = -min(-(GetAzimuthGain(azimuth) + GetElevationGain(elevation)), maxA);
	return (gain + dBiGain);
}

//���վ�Ķ������ߵ�ˮƽ�ʹ�ֱ����
complex<double> AntennaArray::getVerticalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = pow(10., 0.1*GetGain(azimuth, elevation));
	double theta = sin(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}

complex<double>AntennaArray::getHorizontalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = pow(10., 0.1*GetGain(azimuth, elevation));
	double theta = cos(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}

//С��վ������
//С��վˮƽ����Ϊȫ�����ߣ���ֱ���������͸������й�
//3dB�ǣ�40, ������棺20dB
//С��վȫ����������Ϊ5dB
double AntennaArray::GetHeGain(double elevation)
{
	double dBgain = -min(12.0*((elevation - tiltAngle) / 40.)*((elevation - tiltAngle) / 40.), 20.);
	dBgain = dBgain + 5;
	return dBgain;
}
//С��վ�Ķ������ߵ�ˮƽ�ʹ�ֱ����
complex<double> AntennaArray::getHeVerticalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = 1;
	double theta = sin(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}

complex<double>AntennaArray::getHeHorizontalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = pow(10., 0.1*GetHeGain(elevation));
	double theta = cos(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}

//�û�Ϊȫ�����ߵ�ˮƽ�ʹ�ֱ����
complex<double> AntennaArray::getominaVerticalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = 1;
	double theta = sin(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}
complex<double>AntennaArray::getominaHorizontalFieldPattern(double azimuth, double elevation, int elementNumber)
{
	double gain = 1;
	double theta = cos(slantAngle[elementNumber] / 180.*PI);
	return complex<double>(sqrt(gain)*theta, 0.0);
}



double AntennaArray::getDistanceLambda(int elementNumber)
{

	return distanceToFirstElement[elementNumber];
}

// ��thetaֵ������(-180, 180)
double AntennaArray::wrapAngleDistribution(double theta)
{
	theta = theta - 360. * floor(theta / 360.);
	return (theta - 360. * floor(theta / 180.));
}