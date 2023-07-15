#include "lte-phy.h"
#include "NetworkNode.h"
#include "lte-bandwidth-manager.h"
#include "transmitted-signal.h"
#include "packet-burst.h"
#include "interference.h"

LtePhy::LtePhy()
{
	m_device = NULL;	
	m_ltebandwidthManager = NULL;
	SetInterference(new Interference());
}

LtePhy::~LtePhy()
{
	m_device = NULL;
	m_ltebandwidthManager = NULL;
}


void
LtePhy::Destroy()
{
	m_device = NULL;	
	m_ltebandwidthManager = NULL;
	delete m_interference;
}


void
LtePhy::SetDevice(NetworkNode* d)
{
	m_device = d;
}

NetworkNode*
LtePhy::GetDevice()
{
	return m_device;
}

void
LtePhy::SetlteBandwidthManager(lte_BandwidthManager* s)
{
	m_ltebandwidthManager = s;
}

lte_BandwidthManager*
LtePhy::GetlteBandwidthManager()
{
	return m_ltebandwidthManager;
}


void
LtePhy::SetInterference(Interference* i)
{
	m_interference = i;
}


Interference*
LtePhy::GetInterference()
{
	return m_interference;
}


void
LtePhy::SetTxPower(double p)
{
	m_txPower = p;
}

double
LtePhy::GetTxPower(void)
{
	return m_txPower;
}