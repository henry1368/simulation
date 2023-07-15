



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
	m_unscheduledEvents = 0;            //���ɺ���
	m_calendar = new Calendar;
	m_uid = 0;

	SetStop(Sim_stop_time);        //�趨�������ʱ��

}

Simulator::~Simulator()
{
	while (!m_calendar->IsEmpty())
	{
		m_calendar->RemoveEvent();      //ɾ��������
	}
	delete m_calendar;
}

double
Simulator::Now()
{
	return m_currentTs;               //����doubleֵ
}


void
Simulator::Run()
{
	/*
	* This method start the whole simulation
	* The simulation will end when no events are into the                  //������ʼ����ֱ��û���¼���������
	* calendar list.
	*/
	m_stop = false;
	while (!m_calendar->IsEmpty() && !m_stop)
	{


		ProcessOneEvent();              //��Ϊ����ûֹͣ����������
	}
}

void
Simulator::ProcessOneEvent()
{
	Event *next = m_calendar->GetEvent();        //ȡ����ǰ�¼�

	--m_unscheduledEvents;                       //δ�����¼���һ
	m_currentTs = next->GetTimeStamp();          //ȡ���¼�ʱ��
	m_currentUid = next->GetUID();               //ȡ��ʱ���UID

	next->RunEvent();                            //���д����¼���ԭ����Ϊ��  

	m_calendar->RemoveEvent();                   //���
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


	//����Ϊ�ڷ�����������ϵͳ��Ϣ�ͷ���ͳ������д��������ļ�


	std::cout << " SIMULATOR_DEBUG: Stop ()"
		<< std::endl;
	m_stop = true;                              //��ǰ״̬Ϊֹͣ
}

void
Simulator::SetStop(double time)
{
	DoSchedule("lte", time,MakeEvent(&Simulator::Stop, this));
}


void
Simulator::DoSchedule(char* attribution, double time,                          //�¼��滮
Event *event)
{
	double timeStamp = time + Now();                         //��ǰʱ��+�¼�ʱ��
	event->SetTimeStamp(timeStamp);                          //�趨
	event->SetAttribution(attribution);

	++m_unscheduledEvents;                                  //�¼�+1

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
	return m_calendar;               //����calendarֵ
}
bool Simulator::ifstop()
{
	return m_stop;               //����boolֵ
}

int Simulator::GetunscheduledEvents()
{

	return m_unscheduledEvents;
}