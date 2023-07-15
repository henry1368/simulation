
#ifndef ANTENNAARRAY_H
#define ANTENNAARRAY_H

#include<vector>
#include <complex>

using namespace std;//不加此句程序会报很多错误

// 天线类

enum enumAntennaType
{
	Antenna_IMTA_bs,
	Antenna_IMTA_ms
};

class AntennaArray
{
public:

	double  dBiGain;      //dBi
	double  theta3dB;      //degree
	double  phi3dB;       //degree
	double  maxA;         //dB 
	double  tiltAngle;     //degree   
	double  Bs_height;     //基站天线高度  
	double  Ut_height;     //用户天线高度
	double broadsideAngle;//宽边角度，即基准方向
	vector<double> slantAngle;
	vector<double> distanceToFirstElement;
	enumAntennaType antennaType;//天线类型，用户还是基站


	AntennaArray();
	~AntennaArray();
	AntennaArray(enumAntennaType type, double broadsideAzimuth, char * str);//不同场景下天线设置

	double getAzimuthRelativeToBroadside(double absoluteAzimuth);

	// 将theta值限制在(-180, 180)
	double wrapAngleDistribution(double theta);
	//计算水平增益:输入为水平角
	double GetAzimuthGain(double azimuth);

	//计算垂直增益：输入为仰角
	double 	GetElevationGain(double elevation);

	//计算天线总增益：输入为水平角和仰角
	double GetGain(double azimuth, double elevation);
	//计算天线垂直极化值
	complex<double> getVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	//计算天线水平极化值
	complex<double>getHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	//得到第m个天线与第一根天线的距离
	double getDistanceLambda(int elementNumber);
	//
	complex<double> getominaVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	//
	complex<double>getominaHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	//小基站
	double GetHeGain(double elevation);
	complex<double> getHeVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	complex<double>getHeHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	/*
	char         nAntenna;         //天线阵组的天线数
	float        antennaDistance;  //天线波长归一化间隔，单位m
	*/

};
#endif
