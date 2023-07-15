#include "dcf-manager.h"
#include "DcfState_Cluster.h"
#include "DcfState.h"
#include "NetworkNode.h"
#include "AP.h"
#include "ap-mac.h"
#include "UserEquipment.h"
#include "ue-mac.h"
#include "simulator.h"
#include "packet-burst.h"
#include "messages.h"
#include "application.h"
//#include "time.h"

DcfState_Cluster::DcfState_Cluster(AP*ap)
{
	  m_state = new vector<DcfState*>;
	  m_lastNavStart=0.0;
	  m_lastNavDuration = 0.0;
	  m_TxStart = 0.0;
	  m_TxDuration = 0.0;
	  m_RxStart = 0.0;
	  m_RxDuration = 0.0;
	  m_RxEnd = 0.0;
	  m_lastRxReceiveOk = 0;
	  local_ap = ap;
}


DcfState_Cluster::~DcfState_Cluster()

{
/*
	vector<DcfState*>::iterator it;
	for (vector<DcfState*>::iterator it = m_state->begin(); it != m_state->end(); it++)       
		delete (*it);
*/
	m_state->clear();
}

void
DcfState_Cluster::StartCop()
{

//	cout << "AP" << local_ap->GetIDNetworkNode() << "�����ڵĴ���ʱ�� t=" <<Simulator::Init()->Now()<< " ��ʼ������"<<endl;

	update_cluster_member();                                     //���¾����б�
	NetworkNode* Tx_Node = Compete();                    //�˱�ʱ϶����С���û��������ݵķ���
	if (Tx_Node == NULL)
	{
//		cout << "HELLO��World" << endl;
		Simulator::Init()->Schedule("wifi", 224, &DcfState_Cluster::StartCop, this); //�����ǰû�з������ݷ���,��ÿ��һ��ʱ϶�ٽ��м��
	}
	else
	{
		double time=0;

         if (Tx_Node->GetNodeType() == NetworkNode::TYPE_UE)
	 {	
			 cout << "�û�" << Tx_Node->GetIDNetworkNode() << "�Ż���ŵ�" << endl;
		//���ɶ�Ӧ����Ϣ
		WIFISendingMessage * s = new WIFISendingMessage();
		s->m_source = Tx_Node;
		s->m_destination = ((UserEquipment*)Tx_Node)->GetTargetAPNode();                           //����Դ��Ŀ�Ľڵ�

		vector<Application*>* m_applist = ((UserEquipment*)Tx_Node)->GetApplist();                  //����ҵ����������ɶ�Ӧ����Ϣ

		double t = Simulator::Init()->Now();                                                        //��ǰʱ��
		double time = ((UserEquipment*)Tx_Node)->GetUeMac()->GetDcfstate()->m_totalbackofftime;       //���û����˱ܽ���

		for (unsigned int i = 0; i < m_applist->size(); i++)
		{
			if (m_applist->at(i)->m_Num_of_net == 1 ||
				(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))    //ѡ��wifi����ҵ���ҷ�FULLBUFFERҵ��
			{

				list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
				list<Application::flowNode *>::iterator it;
				for (it = m_FL->begin(); it != m_FL->end(); it++)
				{
					if ((*it)->m_data_arrive_time <= t + time && (*it)->m_ue_send==false)         //�������е�WIFIҵ��fullbuffer���⣩,��ǰʱ����ǰ��δ����������Ϣ ����Ϣ���Ʒ���
					{
						struct WIFISendingMessage::SendingRecord  R = { i, (*it) };
						s->m_UeWifiSendingMessage->push_back(R);
						(*it)->m_ue_send =true;                                       //������ҵ����Ϣд����Ϣ��������Ѿ�����
					}
					else if ((*it)->m_data_arrive_time > t + time && (*it)->m_ue_send == false)
					{
						break;          //��ҵ�����Ŀ�ܴ�ʱ�����Լ��ٱ�����ʱ��
					}

				}
			}
		}	   
		Simulator::Init()->Schedule("wifi", time+120, &UserEquipment::Wifi_SendPacketBurst, (UserEquipment*)Tx_Node, s);   //�û���������

		cout << "�����˱�" << endl;
		State::iterator it;
		for (it = m_state->begin(); it != m_state->end(); it++)
		{
			if ((*it)->m_totalbackofftime == time)
				(*it)->UpdateFailedState();               //����ܵ��˱�ʱ��һ��������ײ�����ȼ��͵����������˱�(Ĭ����ͬ�˱�ʱ���к��������ȼ���)
			else
			{
               ((*it)->m_totalbackofftime) -= time;                                      //���������û����˱���Ϣ
			   cout << ((*it)->m_totalbackofftime) << endl;
			}
				
		}
		((UserEquipment*)Tx_Node)->GetUeMac()->GetDcfstate()->Is_initial = false;     //������Ϣ���˱�״̬��ʼ��
	 }

		 else if (Tx_Node->GetNodeType() == NetworkNode::TYPE_AP)
		 {
//			 cout << "AP" << Tx_Node->GetIDNetworkNode() << "�Ż���ŵ�" << endl;
			 double time = ((AP*)Tx_Node)->GetApMac()->GetDcfstate()->m_totalbackofftime;       //AP�˱ܽ���ʱ��

			 DcfManager::Init()->GetCluster(local_ap)->m_lastNavStart = Simulator::Init()->Now() + time;  //�ŵ�����"æ"״̬��ʼʱ��Ϊ�˱ܽ���ʱ��

			 Simulator::Init()->Schedule("wifi", time+120, &AP::SendPacketBurst, local_ap);         //ap��������

			 State::iterator it;
			 for (it = m_state->begin(); it != m_state->end(); it++)
			 {
				 if ((*it)->m_totalbackofftime == time)
					 (*it)->UpdateFailedState();               //����ܵ��˱�ʱ��һ��������ײ�����ȼ��͵����������˱�
				 else
					 ((*it)->m_totalbackofftime) -= time;                                      //���������û����˱���Ϣ
			 }

			 ((AP*)Tx_Node)->GetApMac()->GetDcfstate()->Is_initial = false;     //������Ϣ���˱�״̬��ʼ��

		 }

	}

	


}


void
DcfState_Cluster::update_cluster_member()                                              //���¾����б������
{
//	cout << "���¾����û����ϣ�" << endl;
//	srand((unsigned int)time(NULL));
	State* new_mState = new vector<DcfState*>;
	int flag = 0;
	AP::UserEquipmentRecords* records = local_ap->GetUserEquipmentRecords();
	AP::UserEquipmentRecord *record;
	vector<AP::UserEquipmentRecord*>::iterator iter;                             //ȡ�����Ӹ�ap��ÿ���û���¼
	for (iter = records->begin(); iter != records->end(); iter++)
	{
		record = *iter;
		DcfState* ue_dcfstate = record->GetUE()->GetUeMac()->GetDcfstate();      //�û�����Ϣ״̬


		if (ue_dcfstate->Is_initial == false)         //δ��ʼ��ʱ����ҵ����и�ֵ
			ue_dcfstate->ResetState();                   

		if (ue_dcfstate->Is_initial == true && ue_dcfstate->m_accesstime <= Simulator::Init()->Now())     //�ȽϽ���ʱ��͵�ǰ����ʱ��,����ע���п��ܸ��û�û��WIFIҵ�񣬹ʼ�ʹ��ֵҲû�г�ʼ��
		{
			ue_dcfstate->m_accessRequested = 1;                                  //�ó�Ա���뾺��
			new_mState->push_back(ue_dcfstate);                                  //���´س�Ա
		}
/*
		else
			cout << "�û�" << ue_dcfstate->Belong_Node->GetIDNetworkNode() << "�����뾺��"<<endl;
*/

		if (record->GetQueue()->size() > 0)       //AP��Ӧ�Ļ��������������
			        flag = 1;
	}

	if (flag == 1)
	{
	DcfState* ap_dcfstate = local_ap->GetApMac()->GetDcfstate();            //apҲҪ���뾺��
	if (ap_dcfstate->Is_initial == false)
		 ap_dcfstate->ResetState();
	 new_mState->push_back(ap_dcfstate);
	}

//	delete m_state;                                                             //�ͷ��ڴ�
	m_state = new_mState;                                                       //�µĴ�����
}


NetworkNode* 
DcfState_Cluster::Compete()
{
//	cout << "AP" << local_ap->GetIDNetworkNode() << "�Ŷ�Ӧ�Ĵ� ��ʼ����";
	DcfState* Tx_dcfstate;

	if (m_state->size() == 0)           //��ǰû���û�����û������Ҫ���з���

		return NULL;
	else                                
	{
	vector<DcfState*>::iterator it;
	double min_totalbackofftime = 1000000;

	for (it = m_state->begin(); it != m_state->end(); it++)
	{ 
		double totalbackofftime = (*it)->m_totalbackofftime;       //�˱���ʱ�䣺AIFS+BACKOFFBTIME
		//if ((*it)->Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_UE)
		//   cout << " �û�"<<(*it)->Belong_Node->GetIDNetworkNode()<<",���˱�ʱ��Ϊ" << totalbackofftime << endl;
		//else
		//	cout << " AP" << (*it)->Belong_Node->GetIDNetworkNode() << ",���˱�ʱ��Ϊ" << totalbackofftime << endl;

		if (totalbackofftime <= min_totalbackofftime)          //��ͬ���˱�ʱ���Ժ��������
		{
			min_totalbackofftime = totalbackofftime;
			Tx_dcfstate = (*it);                   //�ҵ���Ӧ���˱�ʱ����С���û����������˱ܵ�0��
		}

	}
	return Tx_dcfstate->Belong_Node;               //���������ڵ�
	}

}


void 
DcfState_Cluster::update_cluster_info()            //��������AP��ȫ���û���λ���Լ������л�
{
//	cout << "AP" << local_ap->GetIDNetworkNode() << "�����ڵĴ���ʱ�� t=" << Simulator::Init()->Now() << " ��ʼ����λ�ã�" << endl;

	AP::UserEquipmentRecords* records = local_ap->GetUserEquipmentRecords();
	AP::UserEquipmentRecords *new_records = new AP::UserEquipmentRecords();
	UserEquipment* ue;
	vector<AP::UserEquipmentRecord*>::iterator iter;                             
	for (iter = records->begin(); iter != records->end(); iter++)
	{
		ue = (*iter)->GetUE();           //�û�

		ue->wifi_link_update();                                    //�û�������Ϣ
		if (ue->GetTargetAPNode() != local_ap)
		{
			cout << "�û�" << ue->GetIDNetworkNode() << "�����ӵ�AP�������л�����AP" << local_ap->GetIDNetworkNode() << " ת������" << ue->GetTargetAPNode()->GetIDNetworkNode() << endl;
			/*����û��������л�����
			1. ɾ��ԭAP�й����û��Ļ���
			2. �û��ѷ��͵�δ�յ��������������·���
			*/
			delete(*iter);
			vector<Application*>* m_applist = ue->GetApplist();
			for (unsigned int i = 0; i < m_applist->size(); i++)
			{
				if (m_applist->at(i)->m_Num_of_net == 1 ||
					(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))    //ѡ��wifi����ҵ���ҷ�FULLBUFFERҵ��
				{

					list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
					list<Application::flowNode *>::iterator it;
					for (it = m_FL->begin(); it != m_FL->end(); it++)
					{
						if ((*it)->m_ue_send == true)          //�ѷ��͵���δ�յ��İ�������յ����޳���������Ҫ���½��з���
							(*it)->m_ue_send = false;
						else
							break;                             //�����ı�Ȼȫ��û�з���

					}
				}
			}

			ue->GetUeMac()->GetDcfstate()->Is_initial = false;   //�µ�����AP�����������û��˱�״̬


		}

		else
			new_records->push_back(*iter);             //���û�з����л���
	}

	delete records;
	local_ap->SetUserEquipmentRecords(new_records);                             //�����趨
	Simulator::Init()->Schedule("wifi", 0.0, &DcfState_Cluster::StartCop, this);

}

void 
DcfState_Cluster::EndDCF()
{
	cout << "AP" << local_ap->GetIDNetworkNode() << "�����ڵĴؽ���������ʱ�̣�" << Simulator::Init()->Now() << endl;
	Simulator::Init()->Schedule("wifi", 0.0, &DcfState_Cluster::StartCop, this);   
}
