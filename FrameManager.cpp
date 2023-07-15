

#include "FrameManager.h"
#include "NetworkManager.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "UserEquipment.h"

#include "packet-burst.h"

FrameManager* FrameManager::ptr=NULL;

FrameManager::FrameManager() {

  m_TTICounter = 0;
  Simulator::Init()->Schedule("lte",0.0, &FrameManager::Start, this);      //将开始作为一个事件      
}

FrameManager::~FrameManager()
{}


void
FrameManager::UpdateTTIcounter ()         //更新TTI
{
  m_TTICounter++;
}

unsigned long
FrameManager::GetTTICounter() const
{
  return m_TTICounter;
}

void
FrameManager::Start ()                               //开始
{

  std::cout << " LTE-A仿真现在开始! "<< std::endl;
  Simulator::Init()->Schedule("lte", 0.0, &FrameManager::StartSubframe, this);
}



void
FrameManager::StartSubframe ()                    //开始子帧
{
	UpdateTTIcounter(); 
	if (SCENARIO_type =="TC1")
	{
	vector<HeNodeB*> *m_henbContainer = NetworkManager::Init()->GetHeNodeBContainer();
	std::vector<HeNodeB*>::iterator iter1;
	HeNodeB *m_henb;
	for (iter1 = m_henbContainer->begin(); iter1 != m_henbContainer->end(); iter1++)
	{
		m_henb = *iter1;
		Simulator::Init()->Schedule("lte", 0.0, &HeNodeB::SendPacketBurst, m_henb);
	}


	}
	else
	{
	vector<ENodeB*> *m_enbContainer   = NetworkManager::Init()->GetENodeBContainer();
	std::vector<ENodeB*>::iterator iter;
	ENodeB *m_enb;
	for (iter = m_enbContainer->begin(); iter != m_enbContainer->end(); iter++)
	{
		m_enb = *iter;
		Simulator::Init()->Schedule("lte", 0.0, &ENodeB::SendPacketBurst, m_enb);
	}

	
    vector<HeNodeB*> *m_henbContainer = NetworkManager::Init()->GetHeNodeBContainer();
	std::vector<HeNodeB*>::iterator iter1;
	HeNodeB *m_henb;
	for (iter1 = m_henbContainer->begin(); iter1 != m_henbContainer->end(); iter1++)
	{
		m_henb = *iter1;
		Simulator::Init()->Schedule("lte", 0.0, &HeNodeB::SendPacketBurst, m_henb);
	}
	}

  Simulator::Init()->Schedule("lte", 1000.0, &FrameManager::StopSubframe, this);       
}//将1ms后的停止作为事件

void
FrameManager::StopSubframe ()
{
//	NetworkManager::Init()->updateLinkInfo();                                     //更新位置LTE系统，更新连接信息
	Simulator::Init()->Schedule("lte", 0.0, &FrameManager::StartSubframe, this);  //否则继续开始

}


