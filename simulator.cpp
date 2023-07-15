



#include "simulator.h"
#include "make-event.h"
#include "parameter.h"
#include <math.h>
#include <fstream>
#include <list>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "WriteSysDataToFile.h"
#include "NetworkManager.h"
#include "AP.h"
#include "lte-bandwidth-manager.h"
#include "wifi-bandwidth-manager.h"
#include "HeNodeB.h"
#include "UserEquipment.h"

Simulator* Simulator::ptr = NULL;

Simulator::Simulator()
{
	m_stop = false;
	m_currentUid = 0;
	m_currentTs = 0;
	m_unscheduledEvents = 0;            //生成函数
	m_calendar = new Calendar;
	m_uid = 0;

	SetStop(Sim_stop_time);        //设定仿真结束时间

}

Simulator::~Simulator()
{
	while (!m_calendar->IsEmpty())
	{
		m_calendar->RemoveEvent();      //删除链表结点
	}
	delete m_calendar;
}

double
Simulator::Now()
{
	return m_currentTs;               //返回double值
}


void
Simulator::Run()
{
	/*
	* This method start the whole simulation
	* The simulation will end when no events are into the                  //整个开始运行直至没有事件进入链表
	* calendar list.
	*/
	m_stop = false;
	while (!m_calendar->IsEmpty() && !m_stop)
	{


		ProcessOneEvent();              //不为空且没停止，处理事情
	}
}

void
Simulator::ProcessOneEvent()
{
	Event *next = m_calendar->GetEvent();        //取出当前事件

	--m_unscheduledEvents;                       //未处理事件减一
	m_currentTs = next->GetTimeStamp();          //取出事件时间
	m_currentUid = next->GetUID();               //取出时间的UID

	next->RunEvent();                            //运行处理事件，原函数为空  

	m_calendar->RemoveEvent();                   //清除
}

int
Simulator::GetUID()
{
	m_uid++;
	return (m_uid - 1);
}

void
Simulator::Stop()
{
	systemDeployData A = { double(int(Sim_stop_time / 1000)), double(nb_totalUe), double(sqrt(3)*0.5 * 500 * 500), double( TX_TotalPower) ,double(LTE_BANDWIDTH),
		double(WIFI_BANDWIDTH), double(WIFI_BANDWIDTH + LTE_BANDWIDTH), double(ap_radius), double(RBs_FOR_LTE), double(nb_HeNodeB_sector),double(nb_cell), double(nb_ue_sector),
		double(RBs_FOR_WIFI), double(Nt), double(Nr), double(SCENARIO), double(Sim_stop_time), double(FeedBackDelay), double(nb_apgrp), double(nb_apingroup), };
    writeSysDeploymentToFile(A);
	vector<AP*> *m_APContainer = NetworkManager::Init()->GetAPContainer();
	for (int i = 0; i < nb_totalAP; i++)
	{
		double x = m_APContainer->at(i)->Getposition()->GetPositionX();
		double y = m_APContainer->at(i)->Getposition()->GetPositionY();
		writeDoubleDataToFile(x);	    
		writeDoubleDataToFile(y);
	}

	vector<HeNodeB*> *m_HeNodeBContainer = NetworkManager::Init()->GetHeNodeBContainer();
	for (int i = 0; i < nb_totalHenb; i++)
	{
		double x = m_HeNodeBContainer->at(i)->Getposition()->GetPositionX();
		double y = m_HeNodeBContainer->at(i)->Getposition()->GetPositionY();
		writeDoubleDataToFile(x);
		writeDoubleDataToFile(y);
	}

	vector<UserEquipment*> *m_UserEquipmentContainer = NetworkManager::Init()->GetUserEquipmentContainer();
	for (int i = 0; i < nb_totalUe; i++)
	{
		double x = m_UserEquipmentContainer->at(i)->Getposition()->GetPositionX();
		double y = m_UserEquipmentContainer->at(i)->Getposition()->GetPositionY();
		writeDoubleDataToFile(x);
		writeDoubleDataToFile(y);
	}


	for (int i = 0; i < int(Sim_stop_time / 1000); i++)
	{
		for (int j = 0; j < nb_totalUe; j++)
		{
			double x = m_UserEquipmentContainer->at(j)->LTE_Receive_Data[i];
			writeDoubleDataToFile(x);
		}

		for (int j = 0; j < nb_totalUe; j++)
		{
			double x = m_UserEquipmentContainer->at(j)->WIFI_Receive_Data[i];
			writeDoubleDataToFile(x);
		}


	}


	//以上为在仿真结束后将相关系统信息和仿真统计数据写入二进制文件


	std::cout << " SIMULATOR_DEBUG: Stop ()"
		<< std::endl;
	m_stop = true;                              //当前状态为停止
}

void
Simulator::SetStop(double time)
{
	DoSchedule("lte", time,MakeEvent(&Simulator::Stop, this));
}


void
Simulator::DoSchedule(char* attribution, double time,                          //事件规划
Event *event)
{
	double timeStamp = time + Now();                         //当前时间+事件时间
	event->SetTimeStamp(timeStamp);                          //设定
	event->SetAttribution(attribution);

	++m_unscheduledEvents;                                  //事件+1

	m_calendar->InsertEvent(event);
}

void
Simulator::PrintMemoryUsage()
{
	//system();
	printf("pmap `ps aux | grep LTE  | grep -v grep | awk '{print $2}'` | grep total\n");
}

Calendar *
Simulator::GetCalendar()
{
	return m_calendar;               //返回calendar值
}
bool Simulator::ifstop()
{
	return m_stop;               //返回bool值
}

int Simulator::GetunscheduledEvents()
{

	return m_unscheduledEvents;
}