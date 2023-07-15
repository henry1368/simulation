#include "wifi-phy.h"
#include "NetworkNode.h"
#include "wifi-bandwidth-manager.h"
#include "transmitted-signal.h"
#include "packet-burst.h"
#include "interference.h"


WifiPhy::WifiPhy()
{
	m_device = NULL;
	m_wifibandwidthManager = NULL;
	SetInterference(new Interference());
}

WifiPhy::~WifiPhy()
{
	m_device = NULL;
    m_wifibandwidthManager = NULL;
}

void
WifiPhy::Destroy()
{
	m_device = NULL;
	m_wifibandwidthManager = NULL;
	delete m_interference;

}


void
WifiPhy::SetDevice(NetworkNode* d)
{
	m_device = d;
}

NetworkNode*
WifiPhy::GetDevice()
{
	return m_device;
}

void
WifiPhy::SetwifiBandwidthManager(wifi_BandwidthManager* s)
{
	m_wifibandwidthManager = s;
	
}

wifi_BandwidthManager*
WifiPhy::GetwifiBandwidthManager()
{
	return m_wifibandwidthManager;
}


void
WifiPhy::SetInterference(Interference* i)
{
	m_interference = i;
}


Interference*
WifiPhy::GetInterference()
{
	return m_interference;
}

void
WifiPhy::SetTxPower(double p)
{
	m_txPower = p;
}

double
WifiPhy::GetTxPower(void)
{
	return m_txPower;
}