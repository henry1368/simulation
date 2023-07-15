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
		int    m_index;                   //�ڵ���
		double   m_data_size;             //���ݵĴ�С  
		double m_data_arrive_time;       //���ݰ�����ʱ��
		bool   m_ue_send;                //�û��Ƿ��ѷ��ͣ��˲�����ҪΪ������WIFI�������û���Ҫ�������������Ϣ�Լ��˱�ʱ����趨
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


//��ǰʱ�䡢ҵ�����͡��û���ҵ��ʼʱ�䡢����ʱ��
Application(ApplicationType m_appType, double startTime, double endTime);

//��������������Ϣ
void DataInsert(flowNode *L, int id, double datasize, double arrivetime);

//ɾ��ָ��ʱ���ڵ����ݰ�
void DeleteApplication(list<flowNode *>App_datalist, double startTime, double endTime);

//ҵ�����ͱ仯
void ChangeApplication(list <flowNode *>App_datalist, double startTime, double endTime, ApplicationType m_appType);

list<flowNode *>* GetDatalist();

//�趨0��1��2�ֱ�Ϊҵ���ѡ����������
/*
0: Lte���磻
1��Wifi���磻
2��Lte��Wifi���繲ͬ����
*/

int m_Num_of_net;  

long double * m_Fountain_data; //���ָ��ֻ������Ȫ��ҵ���м�¼�յ�����������
int * m_flag_ptr;       //���ָ��ֻ������Ȫ��ҵ���жԽ��յ������ݰ��Ƿ���н������λ�������СΪ��Ȫ��ҵ������ݰ�����


private:
	ApplicationType  appType;
	list<flowNode *>* App_datalist;

};

#endif