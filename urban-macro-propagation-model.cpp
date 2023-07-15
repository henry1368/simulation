#include "urban-macro-propagation-model.h"
#include "lte-propagation-loss-model.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "LTE-A_channel.h"
#include  "Mobility.h"
#include "AntennaArray.h"

UMa_LtePropagationLossModel::UMa_LtePropagationLossModel()
{
	SetPropagationType(LtePropagationLossModel::TYPE_UMa);
}

UMa_LtePropagationLossModel::~UMa_LtePropagationLossModel()
{}

double 
UMa_LtePropagationLossModel::Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst)       //m_src为用户，m_dst为基站
{
	if (m_dst->GetNodeType() == NetworkNode::TYPE_ENODEB)     //大基站
	{
		const double c = 300000000;
		const double BS_HEIGHT = 25.0;
		const double MS_HEIGHT = 1.5;
		const double W = 20.0;    //street width
		const double h = 20.0;   //average builing height
		double fc = 2.0;
		E_medi.fc_m = fc;
		const double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c; //break point distance

	}

	return 0;
}

double
UMa_LtePropagationLossModel::Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst)        //阴影衰落
{
	return 0;
}

double
UMa_LtePropagationLossModel::Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst)     //天线增益
{
	return 0;
}

double 
UMa_LtePropagationLossModel::Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst)
{
	return 0;
}