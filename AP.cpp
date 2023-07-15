
#include "NetworkManager.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "AP.h"
#include "wifi-bandwidth-manager.h"
#include "ap-wifi-phy.h"
#include  "ap-mac.h"
#include "wifi-phy.h" 
#include "packet-burst.h"
#include "MacQueue.h"
#include "messages.h"
#include "ue-wifi-phy.h"
#include "parameter.h"
#include "DcfState_Cluster.h"
#include "CodeBook.h"
#include "simulator.h"
#include "dcf-manager.h"
#include "application.h"
#include <list>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


AP::AP()
{}


AP::AP(int idElement, int id_sector,Position * pos)
{
	SetNodeType(TYPE_AP);
	SetIDNetworkNode(idElement);
	SetIndex_sector(id_sector);
	Setposition(pos);

	CreateUserEquipmentRecords();

//	SetAntenna(new AntennaArray("AP"));      //����ap����������

	APWifiPhy *phy = new APWifiPhy();         //wifi�����
	phy->SetDevice(this);
	SetWifiPhy(phy);

	ApMac *mac = new ApMac();          //mac��
	mac->SetDevice(this);
	SetApMac(mac);
}

AP::~AP()
{
	Destroy();
	DeleteUserEquipmentRecords();
    delete  m_apwifiphy;
	delete  m_apmac;


}

void
AP::SetIndex_sector(int id_sector)
{
	Index_sector = id_sector;
}

int
AP::GetIndex_sector()
{
	return Index_sector;
}

void
AP::RegisterUserEquipment(UserEquipment *UE)
{
	UserEquipmentRecord *record = new UserEquipmentRecord(UE);
	GetUserEquipmentRecords()->push_back(record);
}

void
AP::DeleteUserEquipment(UserEquipment *UE)
{
	UserEquipmentRecords *records = GetUserEquipmentRecords();
	UserEquipmentRecord *record;
	UserEquipmentRecords::iterator iter;

	UserEquipmentRecords *new_records = new UserEquipmentRecords();

	for (iter = records->begin(); iter != records->end(); iter++)
	{
		record = *iter;
		if (record->GetUE()->GetIDNetworkNode() != UE->GetIDNetworkNode())
		{
			new_records->push_back(record);
		}
		else    
		{
			delete record;
		}
	}

	m_userEquipmentRecords->clear();
	delete m_userEquipmentRecords;                       //ɾ��ԭ�еģ����¸�ֵ
	m_userEquipmentRecords = new_records;
}



void
AP::CreateUserEquipmentRecords()
{
	m_userEquipmentRecords = new UserEquipmentRecords();
}

void
AP::DeleteUserEquipmentRecords()
{
	UserEquipmentRecords::iterator iter;
	for (iter = m_userEquipmentRecords->begin(); iter != m_userEquipmentRecords->end(); iter++)
	{	
			delete *iter;
	}
	m_userEquipmentRecords->clear();
	delete m_userEquipmentRecords;
}

void
AP::SetUserEquipmentRecords(UserEquipmentRecords* v)
{
	m_userEquipmentRecords=v;
}

AP::UserEquipmentRecords*
AP::GetUserEquipmentRecords()
{
	return m_userEquipmentRecords;
}

AP::UserEquipmentRecord*
AP::GetUserEquipmentRecord(int idUE)
{
	UserEquipmentRecords *records = GetUserEquipmentRecords();
	UserEquipmentRecord *record;
	UserEquipmentRecords::iterator iter;

	for (iter = records->begin(); iter != records->end(); iter++)
	{
		record = *iter;
		if (record->GetUE()->
			GetIDNetworkNode() == idUE)
		{
			return record;
		}
	}
	return false;
}


AP::UserEquipmentRecord::UserEquipmentRecord()
{
	m_UE = NULL;
	//Create initial CQI values:
	m_cqiFeedback.clear();

}

AP::UserEquipmentRecord::~UserEquipmentRecord()
{
	m_cqiFeedback.clear();
}


AP::UserEquipmentRecord::UserEquipmentRecord(UserEquipment *UE)
{
	m_UE = UE;
	m_cqiFeedback.clear();
	
	m_macqueue = new std::vector<MacQueue*>;                                  //������������

	unsigned int i;
	for ( i = 0; i < UE->GetApplist()->size(); i++)                         //ÿ�����ж�Ӧһ��ҵ��   //������Ҫ�ж��Ƿ���ѡ���LTE����
	{
		if (UE->GetApplist()->at(i)->GetApptype() == Application::ApplicationType::TYPE_FULLBUFFER)   //fullbufferҵ������Դ�������Ϊһ��ʼAP��֪��Ҫ��������
		{
			MacQueue *m_queue = new MacQueue();
			m_queue->SetAppNum(i);                                            //��¼ҵ����
			list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist();
			list<Application::flowNode *>::iterator it;
			for (it = datalist->begin(); it != datalist->end(); it++)        //ÿ��ҵ��һ������
				m_queue->Enqueue((*it));                                   //����ҵ���������ɶ�Ӧ�Ķ���
			m_macqueue->push_back(m_queue);
		}
	}

	for (int i = 0; i < FeedBackDelay + 1; i++)     //��ʼ���û���cqi/pmi,���￼�Ƿ�����ʱ��������ʱѭ��
	{
		vector<int> temp_vector;
		for (int j = 0; j < RBs_FOR_WIFI; j++)
		{
			temp_vector.push_back(1);
		}
		m_cqiFeedback.push_back(temp_vector);
		m_PMI.push_back(temp_vector);
		temp_vector.clear();
	}

}


void
AP::UserEquipmentRecord::SetUE(UserEquipment *UE)
{
	m_UE = UE;
}

UserEquipment*
AP::UserEquipmentRecord::GetUE() const
{
	return m_UE;
}

void
AP::UserEquipmentRecord::SetCQI(vector<vector<int> > cqi)
{
	m_cqiFeedback = cqi;
}

vector<vector<int> >
AP::UserEquipmentRecord::GetCQI() const
{
	return m_cqiFeedback;
}

void
AP::UserEquipmentRecord::SetPMI(vector<vector<int> > pmi)
{
	m_PMI = pmi;
}

vector<vector<int> >
AP::UserEquipmentRecord::GetPMI() const
{
	return m_PMI;
}

void
AP::UserEquipmentRecord::SetMcs(int mcs)
{
	m_Mcs = mcs;
}

int
AP::UserEquipmentRecord::GetMcs()
{
	return m_Mcs;
}

void 
AP::UserEquipmentRecord::SetQueue(std::vector<MacQueue*> *macqueue)
{
	m_macqueue = macqueue;
}

std::vector<MacQueue*> * 
AP::UserEquipmentRecord::GetQueue()
{
	return m_macqueue;
}




void
AP::SetWifiPhy(APWifiPhy *phy)                                 //���������
{
	m_apwifiphy = phy;
}


APWifiPhy*
AP::GetWifiPhy()
{
	return  m_apwifiphy;
}


void
AP::SetApMac(ApMac *mac)                     //����mac��
{
	m_apmac = mac;
}

ApMac*
AP::GetApMac()
{
	return m_apmac;
}



void 
AP::SendPacketBurst()       //����
{
	

	DcfManager::Init()->GetCluster(this)->m_TxStart = Simulator::Init()->Now();  //��ʼ����ʱ��

	vector<UserEquipment*>* Dst_UEs=UESieving();     

	if (Dst_UEs->size())
	{
		Scheduling_and_DL_RB_Allocation(Dst_UEs);                     //�ٽ�����Դ����

		Precoding_Generate(Dst_UEs);                                 //����Ԥ����

		vector<PacketBurst*>* Tx_Packets = PacketSchedule(Dst_UEs);  //���ݵ��ȵ��û��ͻ��������Ϣ���ɷ������ݰ�      ���ܵ��������ܳ����ܵķ�����������

		cout << "AP" << GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now()
			<< "���û�" << Dst_UEs->at(0)->GetIDNetworkNode() << "����ҵ�����" << endl;

		GetWifiPhy()->StartTx(Dst_UEs, Tx_Packets);                 //�����������ŵ����ɡ�Ԥ�����������
	}

	else
	{
		GetApMac()->GetDcfstate()->ResetState();   //�����˱���Ϣ

		Simulator::Init()->Schedule("wifi", 224, &DcfState_Cluster::StartCop, DcfManager::Init()->GetCluster(this)); //�����ǰû�з������ݷ���,��ÿ��һ��ʱ϶�ٽ��м��

	}
}



void
AP::ReceivePacketBurst(WIFISendingMessage* s)
{
	cout << "AP" << GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << " ��ʼ�������û�" << s->m_source->GetIDNetworkNode()<<"�ŵ���Ϣ��"<<endl;

	UserEquipment* ue = (UserEquipment*)(s->m_source);                    //Դ�ڵ�

    //�����յ����û���Ҫ�İ�����Ϣ��������ݼ��뻺�����
	vector<struct WIFISendingMessage::SendingRecord>::iterator it;
	for (it = s->m_UeWifiSendingMessage->begin(); it != s->m_UeWifiSendingMessage->end(); it++)
	{
		  int k = (*it).m_Numofapplist;      
		//kΪ��Ӧ��ҵ����
		  int flag = 0;          //ȷ���Ƿ��ж�Ӧ��ҵ��
		  std::vector<MacQueue*>* m_macqueue =GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		  vector<MacQueue*>::iterator it1;
	       for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
		   {
			   if ((*it1)->GetAppNum() == k)
			   {
                  (*it1)->Enqueue((*it).m_flowNode);       //�ҵ����û���ҵ�񻺴���У�������ص�������Ϣд��
				   flag = 1;
			   }
				    
		    }	

		 if (flag == 0)                                   //��ʾû�ж�Ӧҵ��Ļ�����У���ʱ�������µĻ������
		 {
			 MacQueue* m_que = new MacQueue();
			 m_que->Enqueue((*it).m_flowNode);                                           //���
			 m_que->SetAppNum(k);                         
			 m_macqueue->push_back(m_que);
		 }

	}
	delete s;


	Simulator::Init()->Schedule("wifi", 60, &DcfState_Cluster::EndDCF, DcfManager::Init()->GetCluster(this)); //��AP���ڵĴؿ�ʼ��һ�εľ���
	

}



vector<UserEquipment*>*              
AP::UESieving()                                                     //�д������ݵ��û�
{       
	 
    vector<UserEquipment*>* UEs = new vector<UserEquipment*>;
	UserEquipmentRecords::iterator it;
	for (it = m_userEquipmentRecords->begin(); it != m_userEquipmentRecords->end(); it++)
	{
		if ((*it)->GetQueue()->size())                               //AP�ж��ڸ��û��Ļ������ݶ��в�Ϊ��
		{
				UEs->push_back((*it)->GetUE());				         //��¼�û�
			  (*it)->GetUE()->GetWifiPhy()->Nss = 1;                 //�趨Ϊ����
		}


	}

	return UEs;
	                                           
}



void
AP::Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs)            //��Դ����
{


    
	//����ĵ���Ϊ��ѡĳһ���û�����ȫƵ���ķ��� 

	//ѡ���ŵ�����û�
	int m_id = m_UEs->at(0)->GetIDNetworkNode();
	int B_cqi = GetUserEquipmentRecord(m_id)->GetCQI().front().at(0);
	vector<UserEquipment*>::iterator it;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		int k = GetUserEquipmentRecord((*it)->GetIDNetworkNode())->GetCQI().front().at(0);  //�û�CQI
		if (k >= B_cqi)
		{
			B_cqi = k;
			m_id = (*it)->GetIDNetworkNode();
		}
	}

	UserEquipment* ue = NetworkManager::Init()->GetUserEquipmentContainer()->at(m_id);
	m_UEs->clear();
	m_UEs->push_back(ue);                   //���������ȵ��û�

/*	ѡ���һ���û�
	UEs1->push_back(UEs ->front()) 
 */	
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
		{
			ue->GetWifiPhy()->m_channelsForRx[0][i] = 1; 
			ue->GetWifiPhy()->num_data_streams[i] = 1;
			GetWifiPhy()->m_num_tx_streams[i] = 1;
		}


    //���û�MIMO�������µĴ������;����
}



void 
AP::Precoding_Generate(vector<UserEquipment*>* m_UEs)
{
	//���ɷ��Ͷ�ÿ���û���Ԥ��������С��ģΪ�� ���������������û���������
	//�����Ԥ�����СΪ��     ���������������û����� 

	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		int sum = 0;
		GetWifiPhy()->precode_matrix[i] = MatrixXcd::Identity(Nt, GetWifiPhy()->m_num_tx_streams[i]);    //��Ԥ����=���û�1Ԥ���� �û�2Ԥ���롭����

		for (unsigned int j = 0; j < m_UEs->size(); j++)
		{
			//�û��ڸ�Ƶ���Ԥ�������ӦΪ�� ���������������û��ڸ�Ƶ���ϵ�������
			int rows = Nt;
			int cols = m_UEs->at(j)->GetWifiPhy()->num_data_streams[i];
			if (cols>0)
			{
			// ����Ӧ���ǲ������Ԥ�����㷨����Ԥ����
		    //�������:	 m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = MatrixXcd::Random(rows, cols);			
			//---------------------------------------------
			int pmi_index = GetUserEquipmentRecord(m_UEs->at(j)->GetIDNetworkNode())->GetPMI().at(0).at(i);
			switch (cols)
			{
			case 1: { m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = CodeBook::Init()->W1[pmi_index]; break; }
			case 2: { m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = CodeBook::Init()->W2[pmi_index]; break; }
			default:break;
			}

			GetWifiPhy()->precode_matrix[i].middleCols(sum, cols) = m_UEs->at(j)->GetWifiPhy()->precode_matrix[i].leftCols(cols);  //ͨ���滻���û�Ԥ��������
			sum += cols;
			}
		}
	}
}


vector<PacketBurst*>* 
AP::PacketSchedule(vector<UserEquipment*>* m_UEs)                           //ȷ�����û��������� 
{
	double Data_rate[10] = { 29.3, 58.5, 87.8, 117, 175.5, 234.0, 263.3, 292.5, 351.0, 390.0 };      //��������

  //����Ϊ�˴α������û���ȡ�����û��Ļ�����Ϣ
	vector<PacketBurst*>* m_SendPackets = new vector<PacketBurst*>;
	vector<UserEquipment*>::iterator it;
	UserEquipment* ue;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		ue = *it;                                                                                      //ÿ���û���Ӧһ��������
		vector<MacQueue*>*m_MacQueue = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		PacketBurst* ue_packets = new PacketBurst();                                                   //ÿ������Ӧһ����ͬ��ҵ�񻺴���� 
		Packet * m_SendPacket;
		for (int i = 0; i < ue->GetWifiPhy()->Nss;i++)                                                
		{	  
			int k = m_MacQueue->at(i)->GetAppNum();                                                    
			int m_Net = ue->GetApplist()->at(k)->m_Num_of_net;                                         //������Ӧҵ������磨�ж��Ƿ�Ϊ��Ȫ�룩

			int m_mcs = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetCQI().at(0).at(0);          //����MCS
			GetUserEquipmentRecord(ue->GetIDNetworkNode())->SetMcs(m_mcs);

			double PPDU_size = Data_rate[m_mcs]*5844-6*8;                                   //����ʱ��Ϊ2332us,��ȥ6���ֽڿ���
			m_SendPacket = m_MacQueue->at(i)->GetPacketToTramsit(m_Net, PPDU_size);                     //ȷ���˴δ���İ� 
			ue_packets->AddPacket(m_SendPacket);
			              
		}   		
		m_SendPackets->push_back(ue_packets);
	}

	return  m_SendPackets;

 //ÿ��AP��Ӧ�������û�Ӧ����һ�����������б���Щ�����б���Ե����յ��û���������������ģ����û������л�ʱ�������ݶ��лᱻ���

}




