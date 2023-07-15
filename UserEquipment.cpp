
#include "UserEquipment.h"
#include "NetworkNode.h"
#include "ENodeB.h"
#include "AP.h"
#include "wifi-bandwidth-manager.h"
#include "lte-bandwidth-manager.h"
#include "wifi-phy.h"
#include "ue-wifi-phy.h"
#include "ap-wifi-phy.h"
#include "LTE-A_channel.h"
#include "WIFI_channel.h"
#include "lte-propagation-loss-model.h"
#include "wifi-propagation-loss-model.h"
#include  "MadridGrid-propagation-model.h"
#include  "urban-micro-propagation-model.h"
#include "lte-phy.h"
#include "ue-lte-phy.h"
#include "henb-lte-phy.h"
#include "enb-lte-phy.h"
#include "ue-mac.h"
#include "Mobility.h"
#include "position.h"
#include "MacQueue.h"
#include "NetworkManager.h"
#include "parameter.h"
#include "math.h"
#include "application.h"
#include "simulator.h"
#include  "application.h"
#include "lte-mac.h"

template < typename T1>
void swap(T1 array[], int i, int j);

void SelectionSort(double array[], int array1[], int n);


UserEquipment::UserEquipment ()
{}

UserEquipment::UserEquipment (int idElement,
	                          Position* position,
							  int id_sector,
							  ENodeB * targetEnbNode,
							  HeNodeB * targetHenbNode,
							  AP * targetAPNode,
							  Mobility *mobility
							  )
{

  SetNodeType(TYPE_UE);                 //�趨����
  SetIDNetworkNode (idElement);
  Setposition(position);
  m_state=1;
  SetIndex_sector(id_sector);
  SetMobilityModel(mobility);


  
  SetTargetEnbNode(targetEnbNode);    //�����Ҳ�ڴ�ʱ����
  SetTargetHenbNode(targetHenbNode);  //�����Ҳ�ڴ�ʱ����
  SetTargetAPNode(targetAPNode);

  SetTargetUe(NULL);
  D2D_Tx_record = NULL;         //Ĭ���Է����û�����
  SetUEType(LTE_Rx);


  UeLtePhy *phy = new UeLtePhy();  //lte�����
  phy->SetDevice(this);
  SetLtePhy(phy);

  UeWifiPhy *wphy = new UeWifiPhy();  //wifi�����
  wphy->SetDevice(this);
  SetWifiPhy(wphy);

  UeMac *mac = new UeMac();          //mac��
  mac->SetDevice(this);
  SetUeMac(mac);

  LteMac *ltemac = new LteMac();  //lte mac
  ltemac->SetDevice(this);
  SetLteMac(ltemac);

  LteChannel* ltechannel = new LteChannel();  //��ʼ��LTE�ŵ�
  SetLteChannel(ltechannel);
  ltechannel->SetUe(this);


  WifiChannel* wifichannel = new WifiChannel();//��ʼ��WIFI�ŵ�
  SetWifiChannel(wifichannel);
  wifichannel->SetUe(this);

  m_Application_list = new vector<Application*>;   //��ʼ���û�ҵ���б�

  Initial_Receive_Data();

}

UserEquipment::~UserEquipment()
{
	m_targetEnbNode = NULL;
	m_targetAP = NULL;                   
	m_mobility = NULL;
	m_targetUe = NULL;
	delete  m_ueltephy;
	delete  m_uewifiphy;
	delete  m_uemac;
    Destroy ();

}


void UserEquipment::lte_link_update()          //���¼����û�����վ���룬ȷ������Ŀ�꣬�ڴ�֮ǰӦ���Ǹ���λ��
{
	if (SCENARIO_type == "TC1")
	{
	Position* pos = Getposition();                    //�û�λ��
	GetMobilityModel()->UpdatePosition(pos);          //�û�����λ��

//�ж��Ƿ���λ�������仯

	Position* last_pos= GetMobilityModel()->m_last_update_lte_position;   //�ϴμ����߶ȵ�λ��:m_last_update_lte_position��SetMobilityModel(Mobility* m)�������Ѿ���ʼ��
	double m_distance = distance(last_pos, pos);                          //�仯����

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)     //λ�þ��뷢�������仯�����¼����û������л�վ�ľ��뼰LOS�����Լ��ŵ�ϵ������߶ȡ�С�߶ȣ���, ���ж��Ƿ���Ҫ�л�
	{
		GetMobilityModel()->m_last_update_lte_position = pos;

		NetworkManager* nm = NetworkManager::Init();
	//	std::vector<ENodeB*>* m_ENodeBContainer = nm->GetENodeBContainer();
		std::vector<HeNodeB*>* m_HeNodeBContainer = nm->GetHeNodeBContainer();

		if (GetUEType() == LTE_Rx)
		{
		//	double best_enb_loss = -100000; int id_target_enb = -1;                     //���������ȫ�������³�ʼ��
			double best_henb_loss = -100000; int id_target_henb = -1;

		//    GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_ENodeBContainer);
			GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_HeNodeBContainer);
			//�û���ÿ�����վ���ŵ���߶Ƚ�������ȷ���ź���ǿ��ǰ������Ϊ���Ŵ���  
			/*
			for (int i = 0; i < nb_totalEnb; i++)
			{
				double temp_eNodeB_loss[nb_totalEnb + 1] = { 0.0 };                 //+1����ΪC++���������СΪ0������
				for (int i = 0; i < nb_totalEnb; i++)                               //���ŵ�˥�����ֵ����ʱ����
				{
					temp_eNodeB_loss[i] = GetLteChannel()->eNodeB_power_loss[i];
					ID_Enb[i] = i;
				}
				SelectionSort(temp_eNodeB_loss, ID_Enb, nb_totalEnb);              //�����ŵ�˥��Ӻõ����Ӧ��վIDҲ�任
				best_enb_loss = temp_eNodeB_loss[0];
				id_target_enb = ID_Enb[0];
			}
			*/
			//�����û���ÿ��С��վ���ŵ���߶Ƚ�������ȷ���ź���ǿ��ǰ������Ϊ���Ŵ���  
			for (int i = 0; i < nb_totalHenb; i++)
			{
				double temp_HeNodeB_loss[nb_totalHenb] = { 0.0 };
				for (int i = 0; i < nb_totalHenb; i++)
				{
					temp_HeNodeB_loss[i] = GetLteChannel()->HeNodeB_power_loss[i];
					ID_Henb[i] = i;
				}
				SelectionSort(temp_HeNodeB_loss, ID_Henb, nb_totalHenb);
				best_henb_loss = temp_HeNodeB_loss[0];
				id_target_henb = ID_Henb[0];
			}

			//ȷ�����Ӷ��󣨱ȽϺ��վ��С��վ�ŵ�������õ������
			//ѡ�����Ӻ��վ,����ǰ�����Ļ�վ����ΪС��վ����Ϊ���վ������뵱ǰ��һ��ʱ��Ҫ����ע���ɾ����ǰ��¼
			
				if (GetTargetHenbNode() == NULL || GetTargetHenbNode()->GetIDNetworkNode() != id_target_henb)
				{
					SetTargetHenbNode(m_HeNodeBContainer->at(id_target_henb));
					if (GetTargetEnbNode() != NULL)
						GetTargetEnbNode()->DeleteUserEquipment(this);
				}
			
           /*
			vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
			for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Enb_Select+1�ĺ��վ
			{
				int N = ID_Enb[KK];                             //���վ���
				m_ENodeBs->push_back(m_ENodeBContainer->at(N));
			}
			*/
			vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
			for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Henb_Select+1��С��վ
			{
				int N = ID_Henb[KK];                             //С��վ���
				m_HeNodeBs->push_back(m_HeNodeBContainer->at(N));
			}

		

			
		//	GetLteChannel()->GetLtePropagationLossModel()->calculateNodeB_medi(this, m_ENodeBs);                            //���¼���С�߶��м����
			GetLteChannel()->GetLtePropagationLossModel()->calculateHeNodeB_medi(this, m_HeNodeBs);                         //���¼���С�߶��м����

		

		//	GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));  //���¼���С�߶Ȳ���

		//	delete m_ENodeBs;
			delete m_HeNodeBs;

       }
		else
		{
			//�Ƿ����û�
		}		
	}

      else
	{		
	    //δ�������Ծ���仯�����߶��������¼���
		/*
	     vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
	     for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Enb_Select+1�ĺ��վ
	   {
			int N = ID_Enb[KK];                             //���վ���
			m_ENodeBs->push_back(NetworkManager::Init()->GetENodeBContainer()->at(N));
	    }

		  vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
		 for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Henb_Select+1��С��վ
	   {
			int N = ID_Henb[KK];                             //С��վ���
			m_HeNodeBs->push_back(NetworkManager::Init()->GetHeNodeBContainer()->at(N));
	    }
	
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));
		
		delete m_ENodeBs;
		delete m_HeNodeBs;
		*/
	}
	}
	else
	{
	Position* pos = Getposition();                    //�û�λ��
	GetMobilityModel()->UpdatePosition(pos);          //�û�����λ��

//�ж��Ƿ���λ�������仯

	Position* last_pos= GetMobilityModel()->m_last_update_lte_position;   //�ϴμ����߶ȵ�λ��:m_last_update_lte_position��SetMobilityModel(Mobility* m)�������Ѿ���ʼ��
	double m_distance = distance(last_pos, pos);                          //�仯����

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)     //λ�þ��뷢�������仯�����¼����û������л�վ�ľ��뼰LOS�����Լ��ŵ�ϵ������߶ȡ�С�߶ȣ���, ���ж��Ƿ���Ҫ�л�
	{
		GetMobilityModel()->m_last_update_lte_position = pos;

		NetworkManager* nm = NetworkManager::Init();
		std::vector<ENodeB*>* m_ENodeBContainer = nm->GetENodeBContainer();
		std::vector<HeNodeB*>* m_HeNodeBContainer = nm->GetHeNodeBContainer();

		if (GetUEType() == LTE_Rx)
		{
			double best_enb_loss = -100000; int id_target_enb = -1;                     //���������ȫ�������³�ʼ��
			double best_henb_loss = -100000; int id_target_henb = -1;

		    GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_ENodeBContainer);
			GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_HeNodeBContainer);
			//�û���ÿ�����վ���ŵ���߶Ƚ�������ȷ���ź���ǿ��ǰ������Ϊ���Ŵ���    
			for (int i = 0; i < nb_totalEnb; i++)
			{
				double temp_eNodeB_loss[nb_totalEnb + 1] = { 0.0 };                 //+1����ΪC++���������СΪ0������
				for (int i = 0; i < nb_totalEnb; i++)                               //���ŵ�˥�����ֵ����ʱ����
				{
					temp_eNodeB_loss[i] = GetLteChannel()->eNodeB_power_loss[i];
					ID_Enb[i] = i;
				}
				SelectionSort(temp_eNodeB_loss, ID_Enb, nb_totalEnb);              //�����ŵ�˥��Ӻõ����Ӧ��վIDҲ�任
				best_enb_loss = temp_eNodeB_loss[0];
				id_target_enb = ID_Enb[0];
			}

			//�����û���ÿ��С��վ���ŵ���߶Ƚ�������ȷ���ź���ǿ��ǰ������Ϊ���Ŵ���  
			for (int i = 0; i < nb_totalHenb; i++)
			{
				double temp_HeNodeB_loss[nb_totalHenb] = { 0.0 };
				for (int i = 0; i < nb_totalHenb; i++)
				{
					temp_HeNodeB_loss[i] = GetLteChannel()->HeNodeB_power_loss[i];
					ID_Henb[i] = i;
				}
				SelectionSort(temp_HeNodeB_loss, ID_Henb, nb_totalHenb);
				best_henb_loss = temp_HeNodeB_loss[0];
				id_target_henb = ID_Henb[0];
			}

			//ȷ�����Ӷ��󣨱ȽϺ��վ��С��վ�ŵ�������õ������
			//ѡ�����Ӻ��վ,����ǰ�����Ļ�վ����ΪС��վ����Ϊ���վ������뵱ǰ��һ��ʱ��Ҫ����ע���ɾ����ǰ��¼
			if (best_enb_loss >= best_henb_loss)
			{
				if (GetTargetEnbNode() == NULL || GetTargetEnbNode()->GetIDNetworkNode() != id_target_enb)
				{
					SetTargetEnbNode(m_ENodeBContainer->at(id_target_enb));
					if (GetTargetHenbNode() != NULL)                                //ɾ����¼
						GetTargetHenbNode()->DeleteUserEquipment(this);
				}
			}
			//ѡ������С��վ,����ǰ�����Ļ�վ����Ϊ���վ����ΪС��վ������뵱ǰ��һ��ʱ��Ҫ����ע���ɾ����ǰ��¼
			else
			{
				if (GetTargetHenbNode() == NULL || GetTargetHenbNode()->GetIDNetworkNode() != id_target_henb)
				{
					SetTargetHenbNode(m_HeNodeBContainer->at(id_target_henb));
					if (GetTargetEnbNode() != NULL)
						GetTargetEnbNode()->DeleteUserEquipment(this);
				}
			}

			vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
			for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Enb_Select+1�ĺ��վ
			{
				int N = ID_Enb[KK];                             //���վ���
				m_ENodeBs->push_back(m_ENodeBContainer->at(N));
			}

			vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
			for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Henb_Select+1��С��վ
			{
				int N = ID_Henb[KK];                             //С��վ���
				m_HeNodeBs->push_back(m_HeNodeBContainer->at(N));
			}

		

			
			GetLteChannel()->GetLtePropagationLossModel()->calculateNodeB_medi(this, m_ENodeBs);                            //���¼���С�߶��м����
			GetLteChannel()->GetLtePropagationLossModel()->calculateHeNodeB_medi(this, m_HeNodeBs);                         //���¼���С�߶��м����

		

			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));  //���¼���С�߶Ȳ���

			delete m_ENodeBs;
			delete m_HeNodeBs;

       }
		else
		{
			//�Ƿ����û�
		}		
	}

      else
	{		
	    //δ�������Ծ���仯�����߶��������¼���
		/*
	     vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
	     for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Enb_Select+1�ĺ��վ
	   {
			int N = ID_Enb[KK];                             //���վ���
			m_ENodeBs->push_back(NetworkManager::Init()->GetENodeBContainer()->at(N));
	    }

		  vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
		 for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //ͳ�Ƴ��ź���ǿ��Num_Henb_Select+1��С��վ
	   {
			int N = ID_Henb[KK];                             //С��վ���
			m_HeNodeBs->push_back(NetworkManager::Init()->GetHeNodeBContainer()->at(N));
	    }
	
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));
		
		delete m_ENodeBs;
		delete m_HeNodeBs;
		*/
	}
	
	}	
}




void UserEquipment::wifi_link_update()      //���¼����û���AP�������ȷ������Ŀ�꣬�ڴ�֮ǰӦ���Ǹ���λ��
{

	Position* pos = Getposition();            //��ǰ�û�λ��
	GetMobilityModel()->UpdatePosition(pos);  //�û�����λ��

	Position* last_pos = GetMobilityModel()->m_last_update_wifi_position; //�ϴμ����߶ȵ�λ�ã�m_last_update_wifi_position ��SetMobilityModel(Mobility* m)�������Ѿ���ʼ��
	double m_distance = distance(last_pos, pos);                          //�仯����

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)          //λ�þ��뷢�������仯������д�߶Ⱥ�С�߶ȵ����¼���
	{

		GetMobilityModel()->m_last_update_wifi_position = pos;	
		vector<AP*>* m_APContainer = NetworkManager::Init()->GetAPContainer();
		vector<AP*>* m_APs = NULL;

		if (strcmp(SCENARIO_type, "TC2") == 0)      //����TC2�����������û�ֻ�������¥��һ���wifi AP�����������û�ֻ��ò��wifi AP����
		{
			m_APs = new vector<AP*>;
			vector<AP*>::iterator iter;
			for (iter = m_APContainer->begin(); iter != m_APContainer->end(); iter++)
			{
				double z1 = (*iter)->Getposition()->GetPositionZ();      //AP��Z����
				double z2 = Getposition()->GetPositionZ();
				double z3 = max(z1 - z2, z2 - z1);
				if (z3 < 3.5)                          //��ֱ������ͬһ¥�㷶Χ��  					 
					m_APs->push_back((*iter));
			}
		}
		else
			m_APs = m_APContainer;              //��������ȫ������
		
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_PowerLoss(this, m_APs);

		double best_AP_loss; int id_target_AP;
		double *temp_AP_loss = new double[m_APs->size()];

		for (unsigned int i = 0; i < m_APs->size(); i++)
		{
			ID_AP[i] = m_APs->at(i)->GetIDNetworkNode();
			temp_AP_loss[i] = GetWifiChannel()->AP_power_loss[ID_AP[i]];
		}
		SelectionSort(temp_AP_loss, ID_AP, m_APs->size());
		best_AP_loss = temp_AP_loss[0];
		id_target_AP = ID_AP[0];

		if (this->GetTargetAPNode() != m_APContainer->at(id_target_AP))
			SetTargetAPNode(m_APContainer->at(id_target_AP));                         //ѡ�������ź���ǿ��AP


		//ֻ����ͬȺ���ţ����ֻ���㵽��Ⱥ������С�߶Ȳ���
		int k = 1;
		for (int i = (id_target_AP / nb_apingroup)* nb_apingroup; i < (id_target_AP / nb_apingroup + 1)* nb_apingroup; i++)  //ȷ��������AP��ͬȺ������AP
		{		          
			if (id_target_AP != i)
			{			
				ID_AP[k] = i;                   //ID_AP
			    k++;
			}
		}

		vector<AP*>* temp_APs = new vector<AP*>;
		for (int i = 0; i < nb_apingroup; i++)
		{
			int id = ID_AP[i];
			temp_APs->push_back(m_APContainer->at(id));			
		}
	    

		GetWifiChannel()->GetWifiPropagationLossModel()->calculateAP_medi(this, temp_APs);	     //���¼����߶ȣ��Լ�С�߶ȵ��м����
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_SmallscaleFading(this, temp_APs, (Simulator::Init()->Now() / 1000));

		delete temp_APs;
	}
	else
	{
      //δ�������Ծ���仯�����߶��������¼���
		/*
		vector<AP*>* temp_APs = new vector<AP*>;
		for (int i = 0; i < nb_apingroup; i++)
		{
			int id = ID_AP[i];
			temp_APs->push_back(NetworkManager::Init()->GetAPContainer()->at(id));
		}

		GetWifiChannel()->GetWifiPropagationLossModel()->calculateAP_medi(this, temp_APs);	     //���¼����߶ȣ��Լ�С�߶ȵ��м����
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_SmallscaleFading(this, temp_APs, (Simulator::Init()->Now() / 1000));

		delete temp_APs;
		*/
	}
	
}





void 
UserEquipment::SetIndex_sector(int id_sector)
{
	Index_sector = id_sector;
}

int 
UserEquipment::GetIndex_sector()
{
	return Index_sector;
}


void 
UserEquipment::SetUEType(UeType m)               //�û�����
{
	m_uetype = m;
	if (m != LTE_Rx)
	   GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));    //D2D��·���÷�������Ƶ����Դ
}


UeType 
UserEquipment::GetUEType()
{
	return m_uetype;
}


Mobility*
UserEquipment::GetMobilityModel()             //�ƶ���
{
	return m_mobility;
}


void
UserEquipment::SetMobilityModel(Mobility* m)
{
	m->m_last_update_lte_position = Getposition();
	m->m_last_update_wifi_position = Getposition();   
	m_mobility = m;
}


void UserEquipment::SetTargetEnbNode(ENodeB *n)         //���ӻ�վ
{
	m_targetEnbNode = n;
    if (n != NULL)
	{
	GetLtePhy()->SetlteBandwidthManager(n->GetLtePhy()->GetlteBandwidthManager());	
	 n->RegisterUserEquipment(this);
	}
}

ENodeB *
UserEquipment::GetTargetEnbNode()
{
	return m_targetEnbNode;
}


void UserEquipment::SetTargetHenbNode(HeNodeB *n)         //���ڽ���С��վ
{		
	m_targetHeNbNode = n;
	if (n != NULL)
	{
		GetLtePhy()->SetlteBandwidthManager(n->GetLtePhy()->GetlteBandwidthManager());
		n->RegisterUserEquipment(this);
	}
}

HeNodeB *
UserEquipment::GetTargetHenbNode()
{
	return m_targetHeNbNode;

}


void UserEquipment::SetTargetAPNode(AP* n)                  //����wifi AP
{

	m_targetAP = n;
	if (n!=NULL)
{
	GetWifiPhy()->SetwifiBandwidthManager(n->GetWifiPhy()->GetwifiBandwidthManager());	
	n->RegisterUserEquipment(this);
}

}


AP* UserEquipment::GetTargetAPNode()
{
	return m_targetAP;
}


void UserEquipment::SetTargetUe(UserEquipment* n)   //�����û�
{
	m_targetUe = n;

	if (GetUEType() == D2D_Tx)       //�����ע��D2D_Rx����Ϣ
	{
		RegisterUserEquipment(n);
	}
}


UserEquipment*  
UserEquipment::GetTargetUe()
{
	return m_targetUe;
}


void UserEquipment::SetWifiPhy(UeWifiPhy *phy)      //����wifi�����
{
	m_uewifiphy = phy;
}

UeWifiPhy* 
UserEquipment::GetWifiPhy()
{
	return m_uewifiphy;
}



void UserEquipment::SetLtePhy(UeLtePhy *phy)                     //����Lte�����
{
	m_ueltephy=phy;
}

UeLtePhy* 
UserEquipment::GetLtePhy()
{
	return m_ueltephy;
}

void
UserEquipment::SetLteMac(LteMac *mac)                     //����mac��
{
	m_ltemac = mac;
}

LteMac*
UserEquipment::GetLteMac()
{
	return m_ltemac;
}

void
UserEquipment::SetUeMac(UeMac *mac)                     //����mac��
{
	m_uemac = mac;
}

UeMac*
UserEquipment::GetUeMac()
{
	return m_uemac;
}


LteChannel * 
UserEquipment::GetLteChannel()
{
	return m_lteChannel;
}

void
UserEquipment::SetLteChannel(LteChannel * Pot)    //�û���Ӧ��Lte���ŵ�����
{
	m_lteChannel = Pot;
}

WifiChannel  *
UserEquipment::GetWifiChannel()                   //�û���Ӧ��Wifi���ŵ�����
{
	return m_wifiChannel;
}

void 
UserEquipment::SetWifiChannel(WifiChannel* Pot)
{
	m_wifiChannel = Pot;
}

vector<Application*>* 
UserEquipment::GetApplist()
{
	return m_Application_list;
}

void  
UserEquipment::SetApplist(vector<Application*> *m_applist)     //�û�ҵ��
{
	m_Application_list = m_applist;
}


void 
UserEquipment::NetSelection()                                    //֧�ֶ���ҵ��
{

	//�趨0��1��2�ֱ�Ϊҵ���ѡ����������
	/*
	0: Lte���磻
	1��Wifi���磻
	2��Lte��Wifi���繲ͬ����
	*/

	vector<Application*>::iterator iter;
	for (iter = m_Application_list->begin(); iter != m_Application_list->end(); iter++)
	{
		Application * app =(*iter);
		if ((app->GetApptype() == Application::ApplicationType::TYPE_VOIP) || (app->GetApptype() == Application::ApplicationType::TYPE_WEB))
		{
			     app->m_Num_of_net = 0;                    //�������������ҳҵ���򽫸�ҵ��������վ		
		}

		else if ((app->GetApptype() == Application::ApplicationType::TYPE_VIDEO) || (app->GetApptype() == Application::ApplicationType::TYPE_GAME))
		{
		
			     app->m_Num_of_net = 1;                   //�����ҵ������Ƶ����Ϸҵ�񣬽���ҵ������WIFI���磬��ʱ������ֱ�ӿ���AP,����ͨ���û��������͵���ʽ��AP��
        }
		else if ((app->GetApptype() == Application::ApplicationType::TYPE_FTP) || (app->GetApptype() == Application::ApplicationType::TYPE_FULLBUFFER))

		{
			     app->m_Num_of_net = 2;                  //�����ҵ�������ػ�fullbufferҵ�񣬽���ҵ������WIFI�����LTE���磻	

				 int M = (app->GetDatalist()->back()->m_index)+1;
				 app->m_flag_ptr = new int[M+1];
				 app->m_Fountain_data = new long double[M];
				 for (int i = 0; i < M; i++)
				 {
					 app->m_flag_ptr[i] = 0;            //���ø�ҵ���е��������ı�־��Ϊ0��������Ȫ�������������ͬ����
					 app->m_Fountain_data[i] = 0;     //���ô�����������յ������ݰ�����
				 }


		}

	}

}



void
UserEquipment::Lte_SendPacketBurst(Message* s)                                     //lte����
{
    //������·����
}


void
UserEquipment::Wifi_SendPacketBurst(WIFISendingMessage * s)                                     //�û�Wifi�¸�ap����
{
	//WIFI��������·���û������ŵ����������ڷ������������·����˽�������·���û���������Ϣ����ʽ���з���
	cout << "�û�" << GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << "����ҵ���������Ϣ��" << endl;

	Simulator::Init()->Schedule("wifi", 3, &AP::ReceivePacketBurst, (AP*)s->m_destination, s); //����Ӧ�Ľ����¼����    3us��AP�յ��û���������Ϣ

      
} 


void UserEquipment::Lte_ReceivePacketBurst(PacketBurst* p)
{
	//�հ���ue-lte-phy
}

void UserEquipment::Wifi_ReceivePacketBurst(PacketBurst* p)
{
	//�հ���ue-wifi-phy
}




void D2D_SendPacketBurst(PacketBurst* p);                  //D2D����
void D2D_ReceivePacketBurst(PacketBurst* p);               //D2D�հ�



void UserEquipment::Initial_Receive_Data()
{
	for (int i = 0; i < Sim_stop_time / 1000; i++)          //��ʼ���û���������Ϊ0
	{
		LTE_Receive_Data[i] = 0;
		WIFI_Receive_Data[i] = 0;  
	}
                            
}


void
UserEquipment::RegisterUserEquipment(UserEquipment *UE)
{

	D2D_Tx_record = new UserEquipmentRecord();
	D2D_Tx_record->m_cqiFeedback.clear();
	D2D_Tx_record->m_PMI.clear();

	D2D_Tx_record->m_macqueue = new vector<MacQueue*>;                              //��������
	for (unsigned int i = 0; i<UE->GetApplist()->size(); i++)                       //ÿ�����ж�Ӧһ��ҵ��   //������Ҫ�ж��Ƿ���ѡ���LTE����
	{
		if (UE->GetApplist()->at(i)->m_Num_of_net == 0 || UE->GetApplist()->at(i)->m_Num_of_net == 2)  //��ҵ����LTE����
		{
			MacQueue *m_queue = new MacQueue();
			m_queue->SetAppNum(i);                                            //��¼ҵ����
			list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist();
			list<Application::flowNode *>::iterator it;
			for (it = datalist->begin(); it != datalist->end(); it++)        //ÿ��ҵ��һ������
				m_queue->Enqueue((*it));                                //����ҵ���������ɶ�Ӧ�Ķ���
			D2D_Tx_record->m_macqueue->push_back(m_queue);
		}
	}

	for (int i = 0; i < FeedBackDelay + 1; i++)     //��ʼ���û���cqi/pmi,���￼�Ƿ�����ʱ��������ʱѭ��
	{
		vector<int> temp_vector;
		for (int j = 0; j < RBs_FOR_LTE; j++)
		{
			temp_vector.push_back(int(rand()) % 10);
		}
		D2D_Tx_record->m_cqiFeedback.push_back(temp_vector);
		D2D_Tx_record->m_PMI.push_back(temp_vector);
		temp_vector.clear();
	}
}


void
UserEquipment::DeleteUserEquipment(UserEquipment *UE)
{
	delete D2D_Tx_record;
}



UserEquipment::UserEquipmentRecord::UserEquipmentRecord()
{
	//Create initial CQI values:
	m_cqiFeedback.clear();
	m_PMI.clear();
}

UserEquipment::UserEquipmentRecord::~UserEquipmentRecord()
{
	m_cqiFeedback.clear();
	m_PMI.clear();
}


/*���������������ǽ��������е�����Ԫ�ص�λ��*/

template < typename T1>
void swap(T1 array[], int i, int j)
{
	T1 tmp = array[i];
	array[i] = array[j];
	array[j] = tmp;
}


/*ѡ������*/
void SelectionSort(double array[], int array1[], int n)
{
	for (int i = 0; i<n - 1; i++)
	{
		int biggest = i;
		for (int j = i + 1; j<n; j++)
		{
			if (array[biggest]<array[j])
				biggest = j;
		}
		swap(array, i, biggest);
		swap(array1, i, biggest);
	}
}
