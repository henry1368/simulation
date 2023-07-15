#include<cassert>
#include "WIFI_Channel.h"
#include "string.h"
#include "parameter.h"
#include "wifi-propagation-loss-model.h"


WifiChannel::WifiChannel()
{
	WifiPropagationLossModel* x = new WifiPropagationLossModel();
	SetWifiPropagationLossModel(x);
}


WifiChannel::~WifiChannel()
{}


WifiPropagationLossModel *
WifiChannel::GetWifiPropagationLossModel()
{
	return m_WifiPropagationLossModel;
}


void
WifiChannel::SetWifiPropagationLossModel(WifiPropagationLossModel*b)
{
	m_WifiPropagationLossModel = b;
}


UserEquipment*
WifiChannel::GetUe()
{
	return  m_ue;
}

void
WifiChannel::SetUe(UserEquipment* a)
{
	m_ue = a;
}

