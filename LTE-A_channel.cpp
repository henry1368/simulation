#include<cassert>
#include "LTE-A_channel.h"

LteChannel::LteChannel()
{
	if (strcmp(SCENARIO_type, "UMi") == 0)
	{
		UMi_PropagationLossModel* b = new UMi_PropagationLossModel();
		SetLtePropagationLossModel(b);
	}

	if (strcmp(SCENARIO_type, "UMa") == 0)
	{
		UMi_PropagationLossModel* b = new UMi_PropagationLossModel();
		SetLtePropagationLossModel(b);
	}

	if (strcmp(SCENARIO_type, "TC1") == 0)
	{
		UMi_PropagationLossModel* b = new UMi_PropagationLossModel();
		SetLtePropagationLossModel(b);
	}

	if (strcmp(SCENARIO_type, "TC2") == 0)
	{
		MadridGrid_PropagationLossModel* b = new MadridGrid_PropagationLossModel();
		SetLtePropagationLossModel(b);
	}

}

LteChannel::~LteChannel()
{}


LtePropagationLossModel *
LteChannel::GetLtePropagationLossModel()
{
	return m_LtePropagationLossModel;
}


void 
LteChannel::SetLtePropagationLossModel(LtePropagationLossModel*b)
{
	m_LtePropagationLossModel = b;
}


UserEquipment*
LteChannel::GetUe()
{
	return  m_ue;
}

void 
LteChannel::SetUe(UserEquipment* a)
{
	m_ue = a;
}



