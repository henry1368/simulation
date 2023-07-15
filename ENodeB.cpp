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

  Setposition(s);            //设定位置         

  CreateUserEquipmentRecords();
 
  EnbLtePhy *phy = new EnbLtePhy ();   //phy层
  phy->SetDevice (this);
  SetLtePhy (phy);

  LteMac *mac = new LteMac();          //mac层
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
ENodeB::RegisterUserEquipment (UserEquipment *UE)                     //向基站注册时，基站会根据用户的业务直接生成对应的缓存数据队列
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
  delete m_userEquipmentRecords;                       //删除原有的，重新赋值
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

  m_macqueue = new vector<MacQueue*>;                                        //建立队列
  for (unsigned int i = 0; i<UE->GetApplist()->size(); i++)                       //每个队列对应一个业务   //这里需要判断是否是选择的LTE网络
  { 
	  if (UE->GetApplist()->at(i)->m_Num_of_net == 0 || UE->GetApplist()->at(i)->m_Num_of_net == 2)  //该业务走LTE网络
	 {
	    MacQueue *m_queue = new MacQueue();
		m_queue->SetAppNum(i);                                            //记录业务数
	    list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist(); 
	    list<Application::flowNode *>::iterator it;
	    for (it = datalist->begin(); it != datalist->end(); it++)        //每个业务一个队列
		         m_queue->Enqueue((*it));                                //根据业务数据生成对应的队列
	    m_macqueue->push_back(m_queue);
	  }
  }

  for (int i = 0; i < FeedBackDelay + 1; i++)     //初始化用户的cqi/pmi,这里考虑反馈延时，生成延时循环
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
ENodeB::SetLtePhy(EnbLtePhy *phy)                                 //关于Lte物理层
{
	m_enbltephy = phy;
}


EnbLtePhy*
ENodeB::GetLtePhy()
{
	return  m_enbltephy;
}

void
ENodeB::SetLteMac(LteMac *mac)                     //关于mac层
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
	cout << " ENodeB " << GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << "给用户发送业务包：" << endl;
	if (m_userEquipmentRecords->size() > 0)
	{
		vector<UserEquipment*>* Dst_UEs = UESieving();                //选出当前有数据待发的用户
		Scheduling_and_DL_RB_Allocation(Dst_UEs,GetLtePhy());                      //再进行资源分配
		Precoding_Generate(Dst_UEs);                                   //生成预编码
		vector<PacketBurst*>* Tx_Packets = PacketSchedule(Dst_UEs);    //根据调度的用户和缓存队列信息生成发送数据包              （总的流数不能超过总的发射天线数）

		GetLtePhy()->StartTx(Dst_UEs, Tx_Packets);                     //发包（包括信道生成）

		//以下为打印部分
		cout << " ENodeB " << GetIDNetworkNode() << "号发送目的用户为:" << endl;
		UserEquipment* ue;
		vector<UserEquipment*>::iterator iter;
		for (iter = Dst_UEs->begin(); iter != Dst_UEs->end(); iter++)
		{
			ue = *iter;
			cout << "用户" << ue->GetIDNetworkNode() << ",流数为" << ue->GetLtePhy()->Nss << endl;
		}
	}
	else
	{
	//	cout << " Hello World " << endl;
	}
}



vector<UserEquipment*>*            
ENodeB::UESieving()                            //筛选出当前有数据待发的用户
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
		    (*it)->GetUE()->GetLtePhy()->Nss = 1;    //设定单流
		}

	}


	return UEs;

}








vector<PacketBurst*>*
ENodeB::PacketSchedule(vector<UserEquipment*>* m_UEs)                           //确定给用户所发数据 
{
	//输入为此次被调度用户，取出该用户的缓存信息
	vector<PacketBurst*>* m_SendPackets = new vector<PacketBurst*>;
	vector<UserEquipment*>::iterator it;
	UserEquipment* ue;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		ue = *it;                                                                                      //每个用户对应一个数据流
		vector<MacQueue*>*m_MacQueue = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		PacketBurst* ue_packets = new PacketBurst();                                                   //每个流对应一个缓存队列 
		Packet * m_SendPacket;
		for (int i = 0; i < ue->GetLtePhy()->Nss; i++)                                                  //几个流就发了几个包
		{
			int k = m_MacQueue->at(i)->GetAppNum();
			int m_Net = ue->GetApplist()->at(k)->m_Num_of_net;                                         //该流对应业务的网络（判断是否为喷泉码）
			
			int m_mcs = ue->GetLteMac()->GetAmcModule()->GetMCSFromCQI(GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetCQI().at(0).at(0));  //计算MCS
			GetUserEquipmentRecord(ue->GetIDNetworkNode())->SetMcs(m_mcs);

			int tb_size = ue->GetLteMac()->GetAmcModule()->GetTBSizeFromMCS(m_mcs, ue->GetLtePhy()->m_channelsForRx[0],RBs_FOR_LTE);

			m_SendPacket = m_MacQueue->at(i)->GetPacketToTramsit(m_Net, tb_size);                            //确定此次传输的包  
			ue_packets->AddPacket(m_SendPacket);
		}

		m_SendPackets->push_back(ue_packets);
	}

	return  m_SendPackets;

	//每个enb对应它连接用户应存在一个缓存数据列表，这些数据列表针对的是收到用户的数据请求后建立的，在用户发生切换时，该数据队列会被清空

}

void
ENodeB::Precoding_Generate(vector<UserEquipment*>* m_UEs)
{
	//发端每个用户的预编码矩阵大小为： 发射天线数×该用户在该频点上的数据流数
	//发端预编码大小为：              发射天线数×所有用户在该频点上的总流数 
	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		int N = GetLtePhy()->m_num_tx_streams[i];
		if (N>0)
		 GetLtePhy()->precode_matrix[i] = MatrixXcd::Identity(Nt, N);    //总预编码=【用户1预编码 用户2预编码……】初始化

		int sum = 0;
		for (unsigned int j = 0; j < m_UEs->size(); j++)
		{
			int rows = Nt;
			int cols = m_UEs->at(j)->GetLtePhy()->num_data_streams[i];  //该用户在该频点上的数据流数
			if (cols>0)                                                 
		   {			
			Matrix<complex<double>, Nr, Nt> H_ue = m_UEs->at(j)->GetLteChannel()->channel_Ecoefficients[m_UEs->at(j)->GetTargetEnbNode()->GetIDNetworkNode()][i];  //enb到该用户的信道矩阵Hk（Nr×Nt）
			//SVD分解
		/*	JacobiSVD<MatrixXcf> svd(H_ue, ComputeThinU | ComputeThinV);
			MatrixXcf V = svd.matrixV();
			m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = V;   */	
			//-----------------------------------------
			// 这里应该是采用相关预编码算法来求预编码，本程序是随机生成
			 m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = MatrixXcd::Random(rows, cols);
			//---------------------------------------------
			 int pmi_index = GetUserEquipmentRecord(m_UEs->at(j)->GetIDNetworkNode())->GetPMI().at(0).at(i);
			 switch (cols)
			 {
			 case 1: {m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W1[pmi_index]; break; }
			 case 2: { m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W2[pmi_index]; break; }
			 default:break;
			 }
			 GetLtePhy()->precode_matrix[i].middleCols(sum, cols) = m_UEs->at(j)->GetLtePhy()->precode_matrix[i].leftCols(cols);  //通过替换将用户预编码整合成总预编码
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
