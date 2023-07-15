


#include "WifiChannel.h"
#include "transmitted-signal.h"
#include "packet-burst.h"
#include "NetworkNode.h"
#include "wifi-phy.h"
//#include "../core/eventScheduler/simulator.h"
//#include "load-parameters.h"
#include "propagation-loss-model.h"

WifiChannel::WifiChannel()
{
	m_attachedDevices = new std::vector<NetworkNode*>();
	m_propagationLossModel = new PropagationLossModel();
}

WifiChannel::~WifiChannel()
{
	m_attachedDevices->clear();
	delete m_attachedDevices;
	delete m_propagationLossModel;
}


/*
void
WifiChannel::StartTx(PacketBurst* p, TransmittedSignal* txSignal, NetworkNode* src)
{
#ifdef TEST_DEVICE_ON_CHANNEL

	std::cout << "LteChannel::StartTx ch " << GetChannelId() << std::endl;
#endif

	//Simulator::Init()->Schedule(0.001,&WifiChannel::StartRx,this,p,txSignal,src);
#ifdef TRANSMISSION_DEBUG
	std::cout << "   =======  channel  =======" << std::endl;
#endif

}

void
WifiChannel::StartRx(PacketBurst* p, TransmittedSignal* txSignal, NetworkNode* src)
{

	for (std::vector<NetworkNode*>::iterator it = GetDevices()->begin();
		it != GetDevices()->end(); it++)
	{
		NetworkNode* dst = *it;                //dst指向networknode

		//APPLY THE PROPAGATION LOSS MODEL
		TransmittedSignal* rxSignal;
		if (m_propagationLossModel != NULL)
		{
	//		rxSignal = GetPropagationLossModel()->AddLossModel(src, dst, txSignal);  //计算得到接收信号的能量
		}
		else
		{
			rxSignal = txSignal->Copy();
		}

		//DELIVERY THE BURST OF PACKETS
		dst->GetWifiPhy()->StartRx(p->Copy(), rxSignal);
	}

	delete p;
}

*/

void
WifiChannel::AddDevice(NetworkNode* d)
{
	m_attachedDevices->push_back(d);
}

void
WifiChannel::DelDevice(NetworkNode* d)
{
	std::vector<NetworkNode*> *new_list = new std::vector<NetworkNode*>();
	for (std::vector<NetworkNode*>::iterator it = GetDevices()->begin();
		it != GetDevices()->end(); it++)
	{
		NetworkNode* node = *it;

		if (node->GetIDNetworkNode() != d->GetIDNetworkNode())
		{
			new_list->push_back(node);
		}
	}

	m_attachedDevices->clear();
	delete m_attachedDevices;
	m_attachedDevices = new_list;
}

bool
WifiChannel::IsAttached(NetworkNode* d)
{
	for (std::vector<NetworkNode*>::iterator it = GetDevices()->begin();
		it != GetDevices()->end(); it++)
	{
		NetworkNode* node = *it;

		if (node->GetIDNetworkNode() == d->GetIDNetworkNode())
		{
			return true;
		}
	}
	return false;
}

std::vector<NetworkNode*>*
WifiChannel::GetDevices(void)
{
	return m_attachedDevices;
}

void
WifiChannel::SetPropagationLossModel(PropagationLossModel* m)
{
	m_propagationLossModel = m;
}

PropagationLossModel*
WifiChannel::GetPropagationLossModel(void)
{
	return m_propagationLossModel;
}

void
WifiChannel::SetChannelId(int id)
{
	m_channelId = id;
}

int
WifiChannel::GetChannelId(void)
{
	return m_channelId;
}
