
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

//	SetAntenna(new AntennaArray("AP"));      //生成ap的天线阵列

	APWifiPhy *phy = new APWifiPhy();         //wifi物理层
	phy->SetDevice(this);
	SetWifiPhy(phy);

	ApMac *mac = new ApMac();          //mac层
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
	delete m_userEquipmentRecords;                       //删除原有的，重新赋值
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
	
	m_macqueue = new std::vector<MacQueue*>;                                  //建立队列容器

	unsigned int i;
	for ( i = 0; i < UE->GetApplist()->size(); i++)                         //每个队列对应一个业务   //这里需要判断是否是选择的LTE网络
	{
		if (UE->GetApplist()->at(i)->GetApptype() == Application::ApplicationType::TYPE_FULLBUFFER)   //fullbuffer业务特殊对待，即认为一开始AP就知道要发的数据
		{
			MacQueue *m_queue = new MacQueue();
			m_queue->SetAppNum(i);                                            //记录业务数
			list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist();
			list<Application::flowNode *>::iterator it;
			for (it = datalist->begin(); it != datalist->end(); it++)        //每个业务一个队列
				m_queue->Enqueue((*it));                                   //根据业务数据生成对应的队列
			m_macqueue->push_back(m_queue);
		}
	}

	for (int i = 0; i < FeedBackDelay + 1; i++)     //初始化用户的cqi/pmi,这里考虑反馈延时，生成延时循环
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
AP::SetWifiPhy(APWifiPhy *phy)                                 //关于物理层
{
	m_apwifiphy = phy;
}


APWifiPhy*
AP::GetWifiPhy()
{
	return  m_apwifiphy;
}


void
AP::SetApMac(ApMac *mac)                     //关于mac层
{
	m_apmac = mac;
}

ApMac*
AP::GetApMac()
{
	return m_apmac;
}



void 
AP::SendPacketBurst()       //发包
{
	

	DcfManager::Init()->GetCluster(this)->m_TxStart = Simulator::Init()->Now();  //开始传输时间

	vector<UserEquipment*>* Dst_UEs=UESieving();     

	if (Dst_UEs->size())
	{
		Scheduling_and_DL_RB_Allocation(Dst_UEs);                     //再进行资源分配

		Precoding_Generate(Dst_UEs);                                 //生成预编码

		vector<PacketBurst*>* Tx_Packets = PacketSchedule(Dst_UEs);  //根据调度的用户和缓存队列信息生成发送数据包      （总的流数不能超过总的发射天线数）

		cout << "AP" << GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now()
			<< "给用户" << Dst_UEs->at(0)->GetIDNetworkNode() << "发送业务包：" << endl;

		GetWifiPhy()->StartTx(Dst_UEs, Tx_Packets);                 //发包（包括信道生成、预编码的生产）
	}

	else
	{
		GetApMac()->GetDcfstate()->ResetState();   //更新退避信息

		Simulator::Init()->Schedule("wifi", 224, &DcfState_Cluster::StartCop, DcfManager::Init()->GetCluster(this)); //如果当前没有发生数据发送,则每隔一个时隙再进行检测

	}
}



void
AP::ReceivePacketBurst(WIFISendingMessage* s)
{
	cout << "AP" << GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << " 开始接收来用户" << s->m_source->GetIDNetworkNode()<<"号的信息："<<endl;

	UserEquipment* ue = (UserEquipment*)(s->m_source);                    //源节点

    //根据收到的用户需要的包的信息将相关数据加入缓存队列
	vector<struct WIFISendingMessage::SendingRecord>::iterator it;
	for (it = s->m_UeWifiSendingMessage->begin(); it != s->m_UeWifiSendingMessage->end(); it++)
	{
		  int k = (*it).m_Numofapplist;      
		//k为对应的业务编号
		  int flag = 0;          //确定是否有对应的业务
		  std::vector<MacQueue*>* m_macqueue =GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		  vector<MacQueue*>::iterator it1;
	       for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
		   {
			   if ((*it1)->GetAppNum() == k)
			   {
                  (*it1)->Enqueue((*it).m_flowNode);       //找到该用户的业务缓存队列，并将相关的数据信息写入
				   flag = 1;
			   }
				    
		    }	

		 if (flag == 0)                                   //表示没有对应业务的缓存队列，此时则生成新的缓存队列
		 {
			 MacQueue* m_que = new MacQueue();
			 m_que->Enqueue((*it).m_flowNode);                                           //入队
			 m_que->SetAppNum(k);                         
			 m_macqueue->push_back(m_que);
		 }

	}
	delete s;


	Simulator::Init()->Schedule("wifi", 60, &DcfState_Cluster::EndDCF, DcfManager::Init()->GetCluster(this)); //该AP所在的簇开始下一次的竞争
	

}



vector<UserEquipment*>*              
AP::UESieving()                                                     //有待发数据的用户
{       
	 
    vector<UserEquipment*>* UEs = new vector<UserEquipment*>;
	UserEquipmentRecords::iterator it;
	for (it = m_userEquipmentRecords->begin(); it != m_userEquipmentRecords->end(); it++)
	{
		if ((*it)->GetQueue()->size())                               //AP中对于该用户的缓存数据队列不为空
		{
				UEs->push_back((*it)->GetUE());				         //记录用户
			  (*it)->GetUE()->GetWifiPhy()->Nss = 1;                 //设定为单流
		}


	}

	return UEs;
	                                           
}



void
AP::Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs)            //资源分配
{


    
	//这里的调度为单选某一个用户进行全频带的发送 

	//选择信道最好用户
	int m_id = m_UEs->at(0)->GetIDNetworkNode();
	int B_cqi = GetUserEquipmentRecord(m_id)->GetCQI().front().at(0);
	vector<UserEquipment*>::iterator it;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		int k = GetUserEquipmentRecord((*it)->GetIDNetworkNode())->GetCQI().front().at(0);  //用户CQI
		if (k >= B_cqi)
		{
			B_cqi = k;
			m_id = (*it)->GetIDNetworkNode();
		}
	}

	UserEquipment* ue = NetworkManager::Init()->GetUserEquipmentContainer()->at(m_id);
	m_UEs->clear();
	m_UEs->push_back(ue);                   //保留被调度的用户

/*	选择第一个用户
	UEs1->push_back(UEs ->front()) 
 */	
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
		{
			ue->GetWifiPhy()->m_channelsForRx[0][i] = 1; 
			ue->GetWifiPhy()->num_data_streams[i] = 1;
			GetWifiPhy()->m_num_tx_streams[i] = 1;
		}


    //多用户MIMO，多流下的带宽分配;待定
}



void 
AP::Precoding_Generate(vector<UserEquipment*>* m_UEs)
{
	//生成发送端每个用户的预编码矩阵大小规模为： 发射天线数×该用户数据流数
	//发射端预编码大小为：     发射天线数×总用户流数 

	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		int sum = 0;
		GetWifiPhy()->precode_matrix[i] = MatrixXcd::Identity(Nt, GetWifiPhy()->m_num_tx_streams[i]);    //总预编码=【用户1预编码 用户2预编码……】

		for (unsigned int j = 0; j < m_UEs->size(); j++)
		{
			//用户在该频点的预编码矩阵应为： 发射天线数×该用户在该频点上的总流数
			int rows = Nt;
			int cols = m_UEs->at(j)->GetWifiPhy()->num_data_streams[i];
			if (cols>0)
			{
			// 这里应该是采用相关预编码算法来求预编码
		    //随机生成:	 m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = MatrixXcd::Random(rows, cols);			
			//---------------------------------------------
			int pmi_index = GetUserEquipmentRecord(m_UEs->at(j)->GetIDNetworkNode())->GetPMI().at(0).at(i);
			switch (cols)
			{
			case 1: { m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = CodeBook::Init()->W1[pmi_index]; break; }
			case 2: { m_UEs->at(j)->GetWifiPhy()->precode_matrix[i] = CodeBook::Init()->W2[pmi_index]; break; }
			default:break;
			}

			GetWifiPhy()->precode_matrix[i].middleCols(sum, cols) = m_UEs->at(j)->GetWifiPhy()->precode_matrix[i].leftCols(cols);  //通过替换将用户预编码整合
			sum += cols;
			}
		}
	}
}


vector<PacketBurst*>* 
AP::PacketSchedule(vector<UserEquipment*>* m_UEs)                           //确定给用户所发数据 
{
	double Data_rate[10] = { 29.3, 58.5, 87.8, 117, 175.5, 234.0, 263.3, 292.5, 351.0, 390.0 };      //传输速率

  //输入为此次被调度用户，取出该用户的缓存信息
	vector<PacketBurst*>* m_SendPackets = new vector<PacketBurst*>;
	vector<UserEquipment*>::iterator it;
	UserEquipment* ue;
	for (it = m_UEs->begin(); it != m_UEs->end(); it++)
	{
		ue = *it;                                                                                      //每个用户对应一个数据流
		vector<MacQueue*>*m_MacQueue = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		PacketBurst* ue_packets = new PacketBurst();                                                   //每个流对应一个不同的业务缓存队列 
		Packet * m_SendPacket;
		for (int i = 0; i < ue->GetWifiPhy()->Nss;i++)                                                
		{	  
			int k = m_MacQueue->at(i)->GetAppNum();                                                    
			int m_Net = ue->GetApplist()->at(k)->m_Num_of_net;                                         //该流对应业务的网络（判断是否为喷泉码）

			int m_mcs = GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetCQI().at(0).at(0);          //计算MCS
			GetUserEquipmentRecord(ue->GetIDNetworkNode())->SetMcs(m_mcs);

			double PPDU_size = Data_rate[m_mcs]*5844-6*8;                                   //传输时长为2332us,减去6个字节开销
			m_SendPacket = m_MacQueue->at(i)->GetPacketToTramsit(m_Net, PPDU_size);                     //确定此次传输的包 
			ue_packets->AddPacket(m_SendPacket);
			              
		}   		
		m_SendPackets->push_back(ue_packets);
	}

	return  m_SendPackets;

 //每个AP对应它连接用户应存在一个缓存数据列表，这些数据列表针对的是收到用户的数据请求后建立的，在用户发生切换时，该数据队列会被清空

}




