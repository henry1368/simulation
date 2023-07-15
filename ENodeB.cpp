#include "simulator.h"
#include "NetworkNode.h"
#include  "AMCModule.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "enb-lte-phy.h"
#include "lte-mac.h"
#include "lte-phy.h"
#include "lte-bandwidth-manager.h"
#include "LTE-A_channel.h"
#include "packet-burst.h"
#include "MacQueue.h"
#include "application.h"
#include "CodeBook.h"
#include "list"
#include <deque>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

ENodeB::ENodeB ()
{}

ENodeB::ENodeB(int idElement, Position *s)
{
  SetNodeType(TYPE_ENODEB);
  SetIDNetworkNode (idElement);     //id     

  Setposition(s);            //�趨λ��         

  CreateUserEquipmentRecords();
 
  EnbLtePhy *phy = new EnbLtePhy ();   //phy��
  phy->SetDevice (this);
  SetLtePhy (phy);

  LteMac *mac = new LteMac();          //mac��
  mac->SetDevice(this);
  SetLteMac(mac);

  Scheduling_and_DL_RB_Allocation = f;
}


ENodeB::~ENodeB()
{
  Destroy ();
  DeleteUserEquipmentRecords();
  delete  m_enbltephy;
  delete  m_ltemac;
}

void
ENodeB::RegisterUserEquipment (UserEquipment *UE)                     //���վע��ʱ����վ������û���ҵ��ֱ�����ɶ�Ӧ�Ļ������ݶ���
{
  UserEquipmentRecord *record = new UserEquipmentRecord (UE);
  GetUserEquipmentRecords ()->push_back(record);
}

void
ENodeB::DeleteUserEquipment (UserEquipment *UE)
{
  UserEquipmentRecords *records = GetUserEquipmentRecords ();
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  UserEquipmentRecords *new_records = new UserEquipmentRecords ();

  for (iter = records->begin (); iter != records->end (); iter++)
    {
	  record = *iter;
	  if (record->GetUE ()->GetIDNetworkNode () != UE->GetIDNetworkNode ())
	    {
          //records->erase(iter);
          //break;
		  new_records->push_back (record);
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
ENodeB::CreateUserEquipmentRecords ()
{
  m_userEquipmentRecords = new UserEquipmentRecords ();
}

void
ENodeB::DeleteUserEquipmentRecords ()
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
ENodeB::SetUserEquipmentRecords(UserEquipmentRecords* v)
{
	m_userEquipmentRecords = v;
}

ENodeB::UserEquipmentRecords*
ENodeB::GetUserEquipmentRecords ()
{
  return m_userEquipmentRecords;
}

ENodeB::UserEquipmentRecord*
ENodeB::GetUserEquipmentRecord (int idUE)
{
  UserEquipmentRecords *records = GetUserEquipmentRecords ();
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  for (iter = records->begin (); iter != records->end (); iter++)
	{
	  record = *iter;
	  if (record->GetUE ()->
			  GetIDNetworkNode () == idUE)
		{
		  return record;
		}
	}
  return false;
}


ENodeB::UserEquipmentRecord::UserEquipmentRecord ()
{
  m_UE = NULL;
  //Create initial CQI values:
  m_cqiFeedback.clear ();
  m_PMI.clear();
}

ENodeB::UserEquipmentRecord::~UserEquipmentRecord ()
{
  m_cqiFeedback.clear ();
  m_PMI.clear();
}


ENodeB::UserEquipmentRecord::UserEquipmentRecord (UserEquipment *UE)
{
  m_UE = UE;
  m_cqiFeedback.clear ();
  m_PMI.clear();

  m_macqueue = new vector<MacQueue*>;                                        //��������
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
	    m_macqueue->push_back(m_queue);
	  }
  }

  for (int i = 0; i < FeedBackDelay + 1; i++)     //��ʼ���û���cqi/pmi,���￼�Ƿ�����ʱ��������ʱѭ��
  {
	  vector<int> temp_vector;
	  for (int j = 0; j < RBs_FOR_LTE; j++)
	  {
		  temp_vector.push_back(int(rand())%10);
	  }
	  m_cqiFeedback.push_back(temp_vector);
	  m_PMI.push_back(temp_vector);
	  temp_vector.clear();
  }


}

void
ENodeB::UserEquipmentRecord::SetUE (UserEquipment *UE)
{
  m_UE = UE;
}

UserEquipment*
ENodeB::UserEquipmentRecord::GetUE () const
{
  return m_UE;
}

void
ENodeB::UserEquipmentRecord::SetCQI(vector<vector<int> > cqi)
{
  m_cqiFeedback = cqi;
}

vector<vector<int> >
ENodeB::UserEquipmentRecord::GetCQI () const
{
 return m_cqiFeedback;
}

void
ENodeB::UserEquipmentRecord::SetPMI(vector<vector<int> > pmi)
{
	m_PMI = pmi;
}

vector<vector<int> >
ENodeB::UserEquipmentRecord::GetPMI() const
{
	return m_PMI;
}

void
ENodeB::UserEquipmentRecord::SetMcs (int mcs)
{
  m_Mcs = mcs;
}

int
ENodeB::UserEquipmentRecord::GetMcs ()
{
  return m_Mcs;
}

void
ENodeB::UserEquipmentRecord::SetQueue(vector<MacQueue*> *macqueue)
{
	m_macqueue = macqueue;
}

vector<MacQueue*> *
ENodeB::UserEquipmentRecord::GetQueue()
{
	return m_macqueue;
}

void
ENodeB::SetLtePhy(EnbLtePhy *phy)                                 //����Lte�����
{
	m_enbltephy = phy;
}


EnbLtePhy*
ENodeB::GetLtePhy()
{
	return  m_enbltephy;
}

void
ENodeB::SetLteMac(LteMac *mac)                     //����mac��
{
	m_ltemac = mac;
}

LteMac*
ENodeB::GetLteMac()
{
	return m_ltemac;
}



void 
ENodeB::SendPacketBurst()
{
	cout << " ENodeB " << GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << "���û�����ҵ�����" << endl;
	if (m_userEquipmentRecords->size() > 0)
	{
		vector<UserEquipment*>* Dst_UEs = UESieving();                //ѡ����ǰ�����ݴ������û�
		Scheduling_and_DL_RB_Allocation(Dst_UEs,GetLtePhy());                      //�ٽ�����Դ����
		Precoding_Generate(Dst_UEs);                                   //����Ԥ����
		vector<PacketBurst*>* Tx_Packets = PacketSchedule(Dst_UEs);    //���ݵ��ȵ��û��ͻ��������Ϣ���ɷ������ݰ�              ���ܵ��������ܳ����ܵķ�����������

		GetLtePhy()->StartTx(Dst_UEs, Tx_Packets);                     //�����������ŵ����ɣ�

		//����Ϊ��ӡ����
		cout << " ENodeB " << GetIDNetworkNode() << "�ŷ���Ŀ���û�Ϊ:" << endl;
		UserEquipment* ue;
		vector<UserEquipment*>::iterator iter;
		for (iter = Dst_UEs->begin(); iter != Dst_UEs->end(); iter++)
		{
			ue = *iter;
			cout << "�û�" << ue->GetIDNetworkNode() << ",����Ϊ" << ue->GetLtePhy()->Nss << endl;
		}
	}
	else
	{
	//	cout << " Hello World " << endl;
	}
}



vector<UserEquipment*>*            
ENodeB::UESieving()                            //ɸѡ����ǰ�����ݴ������û�
{

	vector<UserEquipment*>* UEs = new vector<UserEquipment*>;
	UserEquipmentRecords::iterator it;
	for (it = m_userEquipmentRecords->begin(); it != m_userEquipmentRecords->end(); it++)
	{
		double m_time = (*it)->GetQueue()->at(0)->GetPacketQueue()->front()->m_flowNode->m_data_arrive_time;

		double t = Simulator::Init()->Now();

		if (m_time <= t)
		{
			UEs->push_back((*it)->GetUE());
		    (*it)->GetUE()->GetLtePhy()->Nss = 1;    //�趨����
		}

	}


	return UEs;

}








vector<PacketBurst*>*
ENodeB::PacketSchedule(vector<UserEquipment*>* m_UEs)                           //ȷ�����û��������� 
{
	//����Ϊ�˴α������û���ȡ�����û��Ļ�����Ϣ
	vector<PacketBurst*>* m_SendPackets = new vector<PacketBurst*>;
	vector<UserEquipment*>::iterator it;
	UserEquipment* ue;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		ue = *it;                                                                                      //ÿ���û���Ӧһ��������
		vector<MacQueue*>*m_MacQueue = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		PacketBurst* ue_packets = new PacketBurst();                                                   //ÿ������Ӧһ��������� 
		Packet * m_SendPacket;
		for (int i = 0; i < ue->GetLtePhy()->Nss; i++)                                                  //�������ͷ��˼�����
		{
			int k = m_MacQueue->at(i)->GetAppNum();
			int m_Net = ue->GetApplist()->at(k)->m_Num_of_net;                                         //������Ӧҵ������磨�ж��Ƿ�Ϊ��Ȫ�룩
			
			int m_mcs = ue->GetLteMac()->GetAmcModule()->GetMCSFromCQI(GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetCQI().at(0).at(0));  //����MCS
			GetUserEquipmentRecord(ue->GetIDNetworkNode())->SetMcs(m_mcs);

			int tb_size = ue->GetLteMac()->GetAmcModule()->GetTBSizeFromMCS(m_mcs, ue->GetLtePhy()->m_channelsForRx[0],RBs_FOR_LTE);

			m_SendPacket = m_MacQueue->at(i)->GetPacketToTramsit(m_Net, tb_size);                            //ȷ���˴δ���İ�  
			ue_packets->AddPacket(m_SendPacket);
		}

		m_SendPackets->push_back(ue_packets);
	}

	return  m_SendPackets;

	//ÿ��enb��Ӧ�������û�Ӧ����һ�����������б���Щ�����б���Ե����յ��û���������������ģ����û������л�ʱ�������ݶ��лᱻ���

}

void
ENodeB::Precoding_Generate(vector<UserEquipment*>* m_UEs)
{
	//����ÿ���û���Ԥ��������СΪ�� ���������������û��ڸ�Ƶ���ϵ���������
	//����Ԥ�����СΪ��              �����������������û��ڸ�Ƶ���ϵ������� 
	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		int N = GetLtePhy()->m_num_tx_streams[i];
		if (N>0)
		 GetLtePhy()->precode_matrix[i] = MatrixXcd::Identity(Nt, N);    //��Ԥ����=���û�1Ԥ���� �û�2Ԥ���롭������ʼ��

		int sum = 0;
		for (unsigned int j = 0; j < m_UEs->size(); j++)
		{
			int rows = Nt;
			int cols = m_UEs->at(j)->GetLtePhy()->num_data_streams[i];  //���û��ڸ�Ƶ���ϵ���������
			if (cols>0)                                                 
		   {			
			Matrix<complex<double>, Nr, Nt> H_ue = m_UEs->at(j)->GetLteChannel()->channel_Ecoefficients[m_UEs->at(j)->GetTargetEnbNode()->GetIDNetworkNode()][i];  //enb�����û����ŵ�����Hk��Nr��Nt��
			//SVD�ֽ�
		/*	JacobiSVD<MatrixXcf> svd(H_ue, ComputeThinU | ComputeThinV);
			MatrixXcf V = svd.matrixV();
			m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = V;   */	
			//-----------------------------------------
			// ����Ӧ���ǲ������Ԥ�����㷨����Ԥ���룬���������������
			 m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = MatrixXcd::Random(rows, cols);
			//---------------------------------------------
			 int pmi_index = GetUserEquipmentRecord(m_UEs->at(j)->GetIDNetworkNode())->GetPMI().at(0).at(i);
			 switch (cols)
			 {
			 case 1: {m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W1[pmi_index]; break; }
			 case 2: { m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W2[pmi_index]; break; }
			 default:break;
			 }
			 GetLtePhy()->precode_matrix[i].middleCols(sum, cols) = m_UEs->at(j)->GetLtePhy()->precode_matrix[i].leftCols(cols);  //ͨ���滻���û�Ԥ�������ϳ���Ԥ����
			 sum += cols;
			}
		}
	}
}


void
ENodeB::Print ()
{
  cout << " ENodeB object:"
      "\n\t m_idNetworkNode = " << GetIDNetworkNode () <<
	  "\n\t Served Users: " <<
  endl;

  vector<UserEquipmentRecord*>* records = GetUserEquipmentRecords ();
  UserEquipmentRecord *record;
  vector<UserEquipmentRecord*>::iterator iter;
  for (iter = records->begin (); iter != records->end (); iter++)
    {
  	  record = *iter;
	  cout << "\t\t idUE = " << record->GetUE ()->GetIDNetworkNode () << endl;		  
    }
}
