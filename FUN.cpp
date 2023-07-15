#include "FUN.h"
#include "simulator.h"
#include "NetworkNode.h"
#include  "AMCModule.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "enb-lte-phy.h"
#include "lte-mac.h"
#include "lte-phy.h"
#include "lte-bandwidth-manager.h"
#include "packet-burst.h"
#include "MacQueue.h"
#include "application.h"
#include "CodeBook.h"
#include "list"
#include <deque>


void f(vector<UserEquipment*>* m_UEs, EnbLtePhy* X)
{

	memset(X->m_num_tx_streams, 0, sizeof(X->m_num_tx_streams));
	int nbOfRBs = X->GetlteBandwidthManager()->GetDLSubChannels().size();

	for (unsigned int i = 0; i < m_UEs->size(); i++)
	{
		UserEquipment* m_UE = m_UEs->at(i);              //取出每个用户并对每个用户流数Nss进行资源分配
		memset(m_UE->GetLtePhy()->m_channelsForRx, 0, sizeof(m_UE->GetLtePhy()->m_channelsForRx));
		memset(m_UE->GetLtePhy()->num_data_streams, 0, sizeof(m_UE->GetLtePhy()->num_data_streams));

		if (i == 0)
		{

			for (int i = 0; i < nbOfRBs; i++)
				m_UE->GetLtePhy()->m_channelsForRx[0][i] = 1;
		}


	}
			
	

	for (int s = 0; s < nbOfRBs; s++)
	{
		for (unsigned int i = 0; i < m_UEs->size(); i++)
		{
			UserEquipment* m_UE = m_UEs->at(i);
			for (unsigned int j = 0; j < Nr; j++)
			{
				m_UE->GetLtePhy()->num_data_streams[s] += m_UE->GetLtePhy()->m_channelsForRx[j][s];
			}
			X->m_num_tx_streams[s] += m_UE->GetLtePhy()->num_data_streams[s];
		}
	}


	//取出实际被调度的用户
	vector<UserEquipment*>::iterator it;
	for (it = m_UEs->begin(); it != m_UEs->end();)
	{
		int m = 0;
		for (int i = 0; i<RBs_FOR_LTE; i++)
			m += (*it)->GetLtePhy()->num_data_streams[i];

		if (m == 0)
			it = m_UEs->erase(it);
		else
			it++;
	}



}