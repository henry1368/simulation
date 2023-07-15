
#ifndef ANTENNAARRAY_H
#define ANTENNAARRAY_H

#include<vector>
#include <complex>

using namespace std;//���Ӵ˾����ᱨ�ܶ����

// ������

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
	double  Bs_height;     //��վ���߸߶�  
	double  Ut_height;     //�û����߸߶�
	double broadsideAngle;//��߽Ƕȣ�����׼����
	vector<double> slantAngle;
	vector<double> distanceToFirstElement;
	enumAntennaType antennaType;//�������ͣ��û����ǻ�վ


	AntennaArray();
	~AntennaArray();
	AntennaArray(enumAntennaType type, double broadsideAzimuth, char * str);//��ͬ��������������

	double getAzimuthRelativeToBroadside(double absoluteAzimuth);

	// ��thetaֵ������(-180, 180)
	double wrapAngleDistribution(double theta);
	//����ˮƽ����:����Ϊˮƽ��
	double GetAzimuthGain(double azimuth);

	//���㴹ֱ���棺����Ϊ����
	double 	GetElevationGain(double elevation);

	//�������������棺����Ϊˮƽ�Ǻ�����
	double GetGain(double azimuth, double elevation);
	//�������ߴ�ֱ����ֵ
	complex<double> getVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	//��������ˮƽ����ֵ
	complex<double>getHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	//�õ���m���������һ�����ߵľ���
	double getDistanceLambda(int elementNumber);
	//
	complex<double> getominaVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	//
	complex<double>getominaHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	//С��վ
	double GetHeGain(double elevation);
	complex<double> getHeVerticalFieldPattern(double azimuth, double elevation, int elementNumber);
	complex<double>getHeHorizontalFieldPattern(double azimuth, double elevation, int elementNumber);
	/*
	char         nAntenna;         //���������������
	float        antennaDistance;  //���߲�����һ���������λm
	*/

};
#endif
