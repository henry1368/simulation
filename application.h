#ifndef APPLICATION_H_
#define APPLICATION_H_

#include <iostream>
#include <list>
#include <algorithm>
#include <string>
using namespace std;


class Application
{
public:

	typedef struct
	{
		int    m_index;                   //节点编号
		double   m_data_size;             //数据的大小  
		double m_data_arrive_time;       //数据包到达时间
		bool   m_ue_send;                //用户是否已发送，此参数主要为满足在WIFI网络中用户需要发送相关数据信息以及退避时间的设定
	}flowNode;


enum ApplicationType
{
	TYPE_VOIP,                    
	TYPE_VIDEO,                   
	TYPE_FTP,                   
	TYPE_WEB,
	TYPE_GAME,	
	TYPE_FULLBUFFER,               
};
Application();
~Application();
void SetApptype(ApplicationType m_appType);
ApplicationType GetApptype();


//当前时间、业务类型、用户、业务开始时间、结束时间
Application(ApplicationType m_appType, double startTime, double endTime);

//将填入数据流信息
void DataInsert(flowNode *L, int id, double datasize, double arrivetime);

//删除指定时间内的数据包
void DeleteApplication(list<flowNode *>App_datalist, double startTime, double endTime);

//业务类型变化
void ChangeApplication(list <flowNode *>App_datalist, double startTime, double endTime, ApplicationType m_appType);

list<flowNode *>* GetDatalist();

//设定0、1、2分别为业务的选择网络类型
/*
0: Lte网络；
1：Wifi网络；
2：Lte和Wifi网络共同发送
*/

int m_Num_of_net;  

long double * m_Fountain_data; //这个指针只用于喷泉码业务中记录收到的数据总量
int * m_flag_ptr;       //这个指针只用于喷泉码业务中对接收到的数据包是否进行解码后置位（数组大小为喷泉码业务的数据包数）


private:
	ApplicationType  appType;
	list<flowNode *>* App_datalist;

};

#endif