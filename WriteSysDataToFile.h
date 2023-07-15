#ifndef _WRITESYSDATATOFILE_H_
#define _WRITESYSDATATOFILE_H_

#include <iostream>
#include <fstream>

using namespace std;


/*! 系统配置结构体*/
struct systemDeployData
{
	double TTI;            //仿真的TTI的总数
	double userNumber;     //仿真的总的用户数
	double area;           //仿真的面积大小
	double power;          //仿真的功率
	double LTE_BAND;       //LTE带宽
	double WIFI_BAND;      //WIFI带宽
	double BAND;           //总带宽
	//2015-06-20下午添加
	double apRadius;       //AP群半径
	double LTERBs;          //LTE资源块的个数

	double HeNodeSector;    //每个扇区小基站的个数
	double cellNum;         //小区的个数
	double userNumPerCell;  //每个小区的用户数
	double subcarrierNumWIFI;//WIFI的子载波的个数
	double nt;              //发射天线的个数
	double nr;              //用户接收天线的个数
	double Scenario;        //定义的仿真场景
	double simuTime;        //仿真时间
	double feedbackDelay;   //反馈延时
	double numApGroup;      //每个扇区wifi群的数目
	double  numApInGroup;    //每个wifi群里wifi的数目

} ;


void writeSysDeploymentToFile(const systemDeployData sys);
/*
写坐标的顺序是
225WIFI坐标
27小基站坐标
90用户坐标
*/
void writeDoubleDataToFile(const double data);


#endif