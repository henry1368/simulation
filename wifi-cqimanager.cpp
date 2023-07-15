
#include "wifi-cqimanager.h"
#include "NetworkNode.h"

wifi_cqiManager::wifi_cqiManager()
{
	m_device = 0;
}

wifi_cqiManager::~wifi_cqiManager()
{
	m_device = 0;
}

void
wifi_cqiManager::SetDevice(NetworkNode* d)
{
	m_device = d;
}

NetworkNode*
wifi_cqiManager::GetDevice(void)
{
	return m_device;
}



void
wifi_cqiManager::SetSendCqi(bool b)
{
	m_sendCqi = b;
}

bool
wifi_cqiManager::GetSendCqi(void)
{
	return m_sendCqi;
}


void
wifi_cqiManager::SetReportingInterval(int i)
{
	m_reportingInterval = i;
}

int
wifi_cqiManager::GetReportingInterval(void)
{
	return m_reportingInterval;
}

void
wifi_cqiManager::SetLastSent()
{
	//m_lastSent = Simulator::Init()->Now() * 1000; //TTI
}

long int
wifi_cqiManager::GetLastSent(void)
{
	return m_lastSent;
}

bool
wifi_cqiManager::NeedToSendFeedbacks(void)
{
	//  std::cout <<"NeedToSendFeedbacks " << (Simulator::Init ()->Now ()*1000) << GetReportingInterval () << std::endl;

	if (GetSendCqi() && GetReportingInterval() == 1)
	{
		return true;
	}

	else if (GetSendCqi() )//&& ((int)(Simulator::Init()->Now() * 1000) - GetLastSent()) >= GetReportingInterval())
	{
		return true;
	}
	else
	{
		return false;
	}
}
