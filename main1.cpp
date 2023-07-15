#include "init.h"
#include "time.h"
#include "parameter.h"
#include"iostream"
#include "simulator.h"
#include"windows.h"
#include "string.h"
#include"FrameManager.h"
#include "dcf-manager.h"

using namespace std;
void main()
{

	int scenario = 1;
	int TTItime = 5;
	Net_Init(scenario);
	NetworkManager* nm = NetworkManager::Init();
	std::vector<UserEquipment*>*m_UEContainer = nm->GetUserEquipmentContainer();

	UserEquipment* ue;
	//double * PL;

	for (int i = 0; i<nb_totalUe; i++)
	{
		ue = m_UEContainer->at(i);
	
	//���ú����������վ�Ĵ�߶ȼ�С�߶Ȳ���ֵ
	ue->GetLteChannel()->Compute_eNodeBpathloss(scenario);		
	ue->GetLteChannel()->calculateNodeBSmallFading(scenario, TTItime);
		
	//���ú�������С��վ�Ĵ�߶ȼ�С�߶Ȳ���ֵ
		ue->GetLteChannel()->Compute_HeNodeBpathloss(scenario);		
		ue->GetLteChannel()->calculateHeNodeBSmallFading(scenario, TTItime);

	//���ú�������Wifi�Ĵ�߶ȼ�С�߶Ȳ���ֵ
		ue->GetWifiChannel()->Compute_Wifipathloss(scenario);		
		ue->GetWifiChannel()->calculateAPSmallFading(scenario, TTItime);
		/*
		PL = ue->GetWifiChannel()->Wifi_power_loss;
		for (int j = 0; j < nb_totalHe; j++)
			cout << PL[j] << endl;
		*/

	}
}