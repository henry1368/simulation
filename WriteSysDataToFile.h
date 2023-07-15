#ifndef _WRITESYSDATATOFILE_H_
#define _WRITESYSDATATOFILE_H_

#include <iostream>
#include <fstream>

using namespace std;


/*! ϵͳ���ýṹ��*/
struct systemDeployData
{
	double TTI;            //�����TTI������
	double userNumber;     //������ܵ��û���
	double area;           //����������С
	double power;          //����Ĺ���
	double LTE_BAND;       //LTE����
	double WIFI_BAND;      //WIFI����
	double BAND;           //�ܴ���
	//2015-06-20�������
	double apRadius;       //APȺ�뾶
	double LTERBs;          //LTE��Դ��ĸ���

	double HeNodeSector;    //ÿ������С��վ�ĸ���
	double cellNum;         //С���ĸ���
	double userNumPerCell;  //ÿ��С�����û���
	double subcarrierNumWIFI;//WIFI�����ز��ĸ���
	double nt;              //�������ߵĸ���
	double nr;              //�û��������ߵĸ���
	double Scenario;        //����ķ��泡��
	double simuTime;        //����ʱ��
	double feedbackDelay;   //������ʱ
	double numApGroup;      //ÿ������wifiȺ����Ŀ
	double  numApInGroup;    //ÿ��wifiȺ��wifi����Ŀ

} ;


void writeSysDeploymentToFile(const systemDeployData sys);
/*
д�����˳����
225WIFI����
27С��վ����
90�û�����
*/
void writeDoubleDataToFile(const double data);


#endif