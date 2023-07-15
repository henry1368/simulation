#include "Random.h"
#include "application.h"
#include "UserEquipment.h"

#include <iostream>
#include <list>
#include <algorithm>
#include <string>

; using namespace std;

Random Ran;




Application::Application(){};
Application::~Application(){};



void
Application::DataInsert(flowNode *L, int id, double datasize, double arrivetime)
{
	L->m_index = id;
	L->m_data_size = datasize;
	L->m_data_arrive_time = arrivetime;
	L->m_ue_send = false;                  //初始化时均为未发送

}

Application::Application(ApplicationType m_appType, double startTime, double endTime)
{

	m_Num_of_net = -1;             //一开始并未选定网络
	m_flag_ptr = NULL;             //仅用于喷泉码业务
	m_Fountain_data = NULL;      //仅用于喷泉码业务

	SetApptype(m_appType);
	double datasize;
	double arrivetime;
	double arrivetime_interval;
	int i;
	App_datalist = new list <flowNode *>;
	switch (m_appType)
	{
	case ApplicationType::TYPE_VOIP:
	{
									   arrivetime = startTime;
									   bool state = 0; //state为0处于激活状态，为1处于静默状态
									   int count = 0;
									   int i = 0;
									   while (arrivetime < endTime)
									   {
										   flowNode *voip = new flowNode;
										   if (state == 0)
										   {
											   datasize = 32;                         //(byte)
											   arrivetime_interval = 20000;           //间隔	20ms
											   DataInsert(voip, i, datasize*8, arrivetime);
											   App_datalist->push_back(voip);
											   arrivetime += arrivetime_interval;
											   i++;
											   int x = rand() % 1000;
											   if ((x < 990) && (x > 0))
											   {
												   state = 0;
											   }
											   else
											   {
												   state = 1;
											   }
										   }
										   if (state == 1)
										   {
											   datasize = 5;                            //(byte)
											   arrivetime_interval = 160000;           //间隔	160ms
											   if (count == 8)
											   {
												   DataInsert(voip, i, datasize*8, arrivetime);
												   App_datalist->push_back(voip);
												   arrivetime += arrivetime_interval;
												   i++;
												   count = 0;
												   int x = rand() % 1000;
												   if ((x < 990) && (x > 0))
												   {
													   state = 1;
												   }
												   else
												   {
													   state = 0;
												   }
											   }
											   else
											   {
												   count = count + 1;
												   int x = rand() % 1000;
												   if ((x < 990) && (x > 0))
												   {
													   state = 1;
												   }
												   else
												   {
													   state = 0;
													   DataInsert(voip, i, datasize*8, arrivetime);
													   App_datalist->push_back(voip);
													   arrivetime += arrivetime_interval;
													   i++;
												   }
											   }
										   }
										   break;
									   }
	}

	case ApplicationType::TYPE_WEB:
	{

									  arrivetime = startTime;
									  i = 0;
									  int datasize_temp = 0;
									  bool state = 0;             //state为0代表当前数据为页面中的主要对象，为1代表当前页面中的嵌入对象
									  while (arrivetime < endTime)
									  {
										  flowNode *web = new flowNode;
										  if (state == 0)
										  {
											  datasize = int(Ran.LogN(8.35, 1.37));//页面中主要对象的大小（byte）
											  if (datasize < 100)                   //数据包大小满足协议规定的模型的最大最小值
											  {
												  datasize = 100;
											  }
											  else if (datasize > 2000000)
											  {
												  datasize = 2000000;
											  }
											  arrivetime_interval = Ran.Exponential(7.69) * 1000000;   // 间隔
											  DataInsert(web, i, datasize*8, arrivetime);
											  App_datalist->push_back(web);
											  arrivetime += arrivetime_interval;
											  i++;
										  }
										  else if (state == 1)
										  {
											  datasize_temp = int(Ran.LogN(6.17, 2.36));//页面中一个嵌入对象的大小(byte)
											  if (datasize_temp < 50)                   //数据包大小满足协议规定的模型的最大最小值
											  {
												  datasize_temp = 50;
											  }
											  else if (datasize_temp > 2000000)
											  {
												  datasize_temp = 2000000;
											  }
											  datasize = int((Ran.Pareto(1.1, 2, 55) - 2)) * datasize_temp;//页面中嵌入对象大小（共有Ran.Pareto(1.1, 2, 55) - 2)个嵌入对象）
											  arrivetime_interval = Ran.Exponential(0.033) * 1000000;
											  DataInsert(web, i, datasize*8, arrivetime);
											  App_datalist->push_back(web);
											  arrivetime += arrivetime_interval;
											  i++;
										  }
									  };
									  break;
	}

	case  ApplicationType::TYPE_VIDEO:
	{
										 arrivetime = startTime;
										 i = 0;
										 int count = 0;  //用于计数，保证每帧有8个片段，每帧与每帧的间隔为100ms
										 double temp_time = 0;  //计算每帧与每帧间隔
										 while (arrivetime < endTime)
										 {
											 flowNode *video = new  flowNode;
											 if (count == 7)
											 {
												 datasize = int(Ran.Pareto(1.2, 20, 125));  //(byte)
												 arrivetime_interval = 100000 - temp_time;  //保证帧与帧之间的间隔为100ms
												 DataInsert(video, i, datasize*8, arrivetime);
												 App_datalist->push_back(video);
												 arrivetime += arrivetime_interval;
												 i++;
												 count = 0;
												 temp_time = 0;
											 }
											 else
											 {
												 datasize = int(Ran.Pareto(1.2, 20, 125));  //(byte)
												 arrivetime_interval = Ran.Pareto(1.2, 2.5, 12.5) * 1000;
												 DataInsert(video, i, datasize*8, arrivetime);
												 App_datalist->push_back(video);
												 arrivetime += arrivetime_interval;
												 i++;
												 count = count + 1;
												 temp_time = temp_time + arrivetime_interval;
											 }
										 };
										 break;

	}
	case  ApplicationType::TYPE_GAME:
	{
										arrivetime = startTime;
										i = 0;
										while (arrivetime < endTime)
										{
											flowNode *game = new flowNode;
											datasize = int(Ran.FisherTippettRan(120, 36));         //(byte)
											arrivetime_interval = Ran.FisherTippettRan(55, 6) * 1000;
											DataInsert(game, i, datasize*8, arrivetime);
											App_datalist->push_back(game);
											arrivetime += arrivetime_interval;
											i++;
										};
										break;

	}

	case  ApplicationType::TYPE_FTP:
	{
									   arrivetime = startTime;
									   i = 0;
									   while (arrivetime < endTime)
									   {
										   flowNode *ftp = new flowNode;
										   datasize = int(Ran.LogN(14.45, 0.35));                    //(byte)
										   if (datasize > 5000000)                                   //数据包大小满足协议规定的模型的最大最小值
										   {
											   datasize = 5000000;
										   }
										   arrivetime_interval = Ran.Exponential(0.006) * 1000000;
										   DataInsert(ftp, i, datasize*8, arrivetime);
										   App_datalist->push_back(ftp);
										   arrivetime += arrivetime_interval;
										   i++;

									   };
									   break;

	}



	case  ApplicationType::TYPE_FULLBUFFER:
	{
											  arrivetime = startTime;
											  for (int i = 0; i < 10; i++)
											  {
											    flowNode *fullbuffer = new flowNode;
											    datasize = 1e20;                  //字节数设定，需要根据最大传输数据大小可能做出调整										
												DataInsert(fullbuffer, i, datasize*8, arrivetime);
												App_datalist->push_back(fullbuffer);
											  }

											  break;
	}
	}
}


void Application::DeleteApplication(list <flowNode *> App_datalist, double startTime, double endTime)
{//删除业务
	for (list<flowNode *>::iterator iter1 = App_datalist.begin(); iter1 != App_datalist.end(); iter1++)
	{
		if ((*iter1)->m_data_arrive_time > startTime && (*iter1)->m_data_arrive_time < endTime)
		{
			delete *iter1;

			App_datalist.erase(iter1);

			iter1 = App_datalist.begin();
		}

	}
}


void Application::ChangeApplication(list <flowNode *>App_datalist, double startTime, double endTime, ApplicationType m_appType)
{//更换业务
	for (list<flowNode *>::iterator iter = App_datalist.begin(); iter != App_datalist.end(); iter++)
	{
		delete *iter;
	}
	App_datalist.clear();
	Application(m_appType, startTime, endTime);
}


list<Application::flowNode *>*
Application::GetDatalist()
{
	return App_datalist;
}

void
Application::SetApptype(Application::ApplicationType m_appType)
{
	appType = m_appType;
}


Application::ApplicationType
Application::GetApptype()
{
	return appType;
}






/*

if (userstate = 10)
{//业务中断，删除链表
switch (applicationType)
{
case 1:
mylist_voip.clear();
return mylist_voip;
break;
case 2:
mylist_web.clear();
return mylist_web;
break;

case 3:
mylist_video.clear();
return mylist_video;
break;
case 4:
mylist_video.clear();
return mylist_game;
break;
case 5:
mylist_ftp.clear();
return mylist_ftp;
break;
}
}
else
{
}
}

*/