#include "simulator.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "HeNodeB.h"
#include "henb-lte-phy.h"
#include "lte-mac.h"
#include "lte-phy.h"
#include"LTE-A_channel.h"
#include"WIFI_channel.h"
#include "lte-bandwidth-manager.h"
#include "packet-burst.h"
#include "MacQueue.h"
#include "application.h"
#include "CodeBook.h"
#include "list"
#include <deque>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

HeNodeB::HeNodeB()
{}

HeNodeB::HeNodeB(int idElement, int id_sector, Position * pos)
{
	SetNodeType(TYPE_HENODEB);                 //设置类型
	SetIDNetworkNode(idElement);                //id
	SetIndex_sector(id_sector);
	Setposition(pos);                     //设定位置

	CreateUserEquipmentRecords();

	//SetAntenna(new AntennaArray("HeNodeB"));     //生成基站的天线阵列

	HenbLtePhy *phy = new HenbLtePhy();
	phy->SetDevice (this);
	SetLtePhy (phy);

	LteMac *mac = new LteMac();          //mac层
	mac->SetDevice(this);
	SetLteMac(mac);
}


HeNodeB::~HeNodeB()
{
	Destroy();
	DeleteUserEquipmentRecords();
	delete  m_henbltephy;
	delete  m_ltemac;
}

void
HeNodeB::SetIndex_sector(int id_sector)
{
	Index_sector = id_sector;
}

int
HeNodeB::GetIndex_sector()
{
	return Index_sector;
}

void
HeNodeB::RegisterUserEquipment(UserEquipment *UE)                     //向小基站注册时，基站会根据用户的业务直接生成对应的缓存数据队列
{
	UserEquipmentRecord *record = new UserEquipmentRecord(UE);
	GetUserEquipmentRecords()->push_back(record);
}

void
HeNodeB::DeleteUserEquipment(UserEquipment *UE)
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
			//records->erase(iter);
			//break;
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
HeNodeB::CreateUserEquipmentRecords()
{
	m_userEquipmentRecords = new UserEquipmentRecords();
}

void
HeNodeB::DeleteUserEquipmentRecords()
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
HeNodeB::SetUserEquipmentRecords(UserEquipmentRecords* v)
{
	m_userEquipmentRecords = v;
}

HeNodeB::UserEquipmentRecords*
HeNodeB::GetUserEquipmentRecords()
{
	return m_userEquipmentRecords;
}

HeNodeB::UserEquipmentRecord*
HeNodeB::GetUserEquipmentRecord(int idUE)
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
	return nullptr;
}


HeNodeB::UserEquipmentRecord::UserEquipmentRecord()
{
	m_UE = NULL;
	//Create initial CQI values:
	m_cqiFeedback.clear();
	m_PMI.clear();
}

HeNodeB::UserEquipmentRecord::~UserEquipmentRecord()
{
	m_cqiFeedback.clear();
	m_PMI.clear();
}


HeNodeB::UserEquipmentRecord::UserEquipmentRecord(UserEquipment *UE)
{
	m_UE = UE;
	m_cqiFeedback.clear();
	m_PMI.clear();

	m_macqueue = new std::vector<MacQueue*>;                                  //建立队列
	for (unsigned int i = 0; i < UE->GetApplist()->size(); i++)                        //每个队列对应一个业务   //这里需要判断是否是选择的LTE网络
	{
		if (UE->GetApplist()->at(i)->m_Num_of_net == 0 || UE->GetApplist()->at(i)->m_Num_of_net == 2)  //该业务走LTE网络
		{
			MacQueue *m_queue = new MacQueue();
			m_queue->SetAppNum(i);                                            //记录业务数
			list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist();
			std::list<Application::flowNode *>::iterator it;
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
			temp_vector.push_back(int(rand()) % 10);
		}
		m_cqiFeedback.push_back(temp_vector);
		m_PMI.push_back(temp_vector);
		temp_vector.clear();
	}
}

void
HeNodeB::UserEquipmentRecord::SetUE(UserEquipment *UE)
{
	m_UE = UE;
}

UserEquipment*
HeNodeB::UserEquipmentRecord::GetUE() const
{
	return m_UE;
}

void
HeNodeB::UserEquipmentRecord::SetCQI(vector<vector<int> > cqi)
{
	m_cqiFeedback = cqi;
}

vector<vector<int> >
HeNodeB::UserEquipmentRecord::GetCQI() const
{
	return m_cqiFeedback;
}

void
HeNodeB::UserEquipmentRecord::SetPMI(vector<vector<int> > pmi)
{
	m_PMI = pmi;
}

vector<vector<int> >
HeNodeB::UserEquipmentRecord::GetPMI() const
{
	return m_PMI;
}

void
HeNodeB::UserEquipmentRecord::SetMcs(int mcs)
{
	m_Mcs = mcs;
}

int
HeNodeB::UserEquipmentRecord::GetMcs()
{
	return m_Mcs;
}

void
HeNodeB::UserEquipmentRecord::SetQueue(vector<MacQueue*> *macqueue)
{
	m_macqueue = macqueue;
}

vector<MacQueue*> *
HeNodeB::UserEquipmentRecord::GetQueue()
{
	return m_macqueue;
}

void
HeNodeB::SetLtePhy(HenbLtePhy *phy)                                 //关于Lte物理层
{
	m_henbltephy = phy;
}


HenbLtePhy*
HeNodeB::GetLtePhy()
{
	return  m_henbltephy;
}

void
HeNodeB::SetLteMac(LteMac *mac)                     //关于mac层
{
	m_ltemac = mac;
}

LteMac*
HeNodeB::GetLteMac()
{
	return m_ltemac;
}

void
HeNodeB::SendPacketBurst()
{
	cout << "HeNodeB" << GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << "给用户发送业务包：" << endl;
	if (m_userEquipmentRecords->size() > 0)
	{
		vector<UserEquipment*>* Dst_UEs = UESieving();                //进行用户筛选，选出有待传业务的用户
		Scheduling_and_DL_RB_Allocation(Dst_UEs);                      //再进行资源分配
		Precoding_Generate(Dst_UEs);                                 //生成预编码
		vector<PacketBurst*>* Tx_Packets = PacketSchedule(Dst_UEs);    //根据调度的用户和缓存队列信息生成发送数据包      （总的流数不能超过总的发射天线数）

		GetLtePhy()->StartTx(Dst_UEs, Tx_Packets);                     //发包（包括信道生成）

		//以下为打印部分
		cout << " HeNodeB " << GetIDNetworkNode() << "号发送目的用户为:" << endl;
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
HeNodeB::UESieving()            
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
			(*it)->GetUE()->GetLtePhy()->Nss = 1;

		}

	}


	return UEs;
}


void
HeNodeB::Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs)            ///资源分配
{  	
	
	memset(GetLtePhy()->m_num_tx_streams, 0, sizeof(GetLtePhy()->m_num_tx_streams));
	int sumofNss = 0;                                                  //所有流数相加求和

	vector<vector<double> > henb_sEff;
	vector<double> henb_HisThroughput;

	for (unsigned int i = 0; i < m_UEs->size(); i++)
	{
		UserEquipment* m_UE = m_UEs->at(i);                   //取出每个用户并对每个用户流数Nss进行资源分配
		memset(m_UE->GetLtePhy()->m_channelsForRx, 0, sizeof(m_UE->GetLtePhy()->m_channelsForRx));
		memset(m_UE->GetLtePhy()->num_data_streams, 0, sizeof(m_UE->GetLtePhy()->num_data_streams));

		vector<int> ue_cqi = GetUserEquipmentRecord(m_UE->GetIDNetworkNode())->GetCQI().front();//该用户的cqi	
		sumofNss = sumofNss + m_UE->GetLtePhy()->Nss;          //所有流数相加求和
		for (int m = 0; m < m_UE->GetLtePhy()->Nss; m++)       //计算每条流的属性
		{
			vector<double> Nss_sEff;
			int numberOfCqi = ue_cqi.size();
			for (int i = 0; i < numberOfCqi; i++)
			{
				double sEff = m_UE->GetLteMac()->GetAmcModule()->GetEfficiencyFromCQI(ue_cqi.at(i));
				Nss_sEff.push_back(sEff);
			}
			henb_sEff.push_back(Nss_sEff);

			int index = int(Simulator::Init()->Now() / 1000);                     //量化到第几个TTI
			double HistoryThroughput = 0;
			double HistoryAverageThroughput = 0;
			if (index == 0)
			{
				HistoryAverageThroughput = ((double)rand()) / 0x7fff;
			}
			else
			{
				for (int s = 0; s < index+1; s++)
				{
					HistoryThroughput += m_UE->LTE_Receive_Data[s];
				}
				HistoryAverageThroughput = HistoryThroughput / index;
			}
			henb_HisThroughput.push_back(HistoryAverageThroughput);
		}
	}
	//下面进行RB分配   
	int nbOfRBs = GetLtePhy()->GetlteBandwidthManager()->GetDLSubChannels().size();
	vector<vector<double> >  metrics(nbOfRBs, vector<double>(sumofNss));   //创建数据流分配优先级矩阵并初始化
	for (int i = 0; i < nbOfRBs; i++)
	{
		for (int j = 0; j < sumofNss; j++)
		{
			metrics[i][j] = 0;
		}
	}
	//计算优先级矩阵值
	//轮询  
	int round = 0;
	for (int i = 0; i < nbOfRBs; i++)
	{
	if (round == sumofNss)
	{
	round = 0;
	metrics[i][round] = 1;
	}
	else
	{
	metrics[i][round] = 1;
	}
	round = round + 1;
	}
	

	//PF
	/*
	for (int i = 0; i < nbOfRBs; i++)
	{
		for (int j = 0; j < sumofNss; j++)
		{
			metrics[i][j] = (henb_sEff.at(j).at(i) * 180000.) / henb_HisThroughput.at(j);           //确定优先级矩阵
		}
	}
	*/

	//具体RB分配开始开始
	for (int s = 0; s < nbOfRBs; s++)
	{
		double targetMetric = 0;
		int k_choose = 0;                          
		bool RBIsAllocated = false;

		for (int k = 0; k < sumofNss; k++)
		{
			if (metrics[s][k] > targetMetric)
			{
				targetMetric = metrics[s][k];                //选出每个RB上优先级最大的流的索引
				k_choose = k;
			}
		}
		RBIsAllocated = true;
		
		int ue_index, Nss_index;      //ue_index, Nss_index 第ue_index个用户的第Nss_index条数据流
		int sum = 0;
		for (unsigned int i = 0; i < m_UEs->size(); i++)
		{
			if (k_choose < sum + m_UEs->at(i)->GetLtePhy()->Nss)                //确定k_chose属于第几个用户第几条数据流
			{
				ue_index = i;
				Nss_index = k_choose - sum;
				break;
			}
			else
			{
				sum += m_UEs->at(i)->GetLtePhy()->Nss;
			}
		}
		//给流矩阵赋值
		m_UEs->at(ue_index)->GetLtePhy()->m_channelsForRx[Nss_index][s] = 1;
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
			GetLtePhy()->m_num_tx_streams[s] += m_UE->GetLtePhy()->num_data_streams[s];
		}
	}

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







	/*
	HeNodeB *henb = (HeNodeB*)GetLteMac()->GetDevice();


	if (m_UEs->size() > 0)                      //计算每个数据流的数据大小、频谱效率、cqi等(对数据流进行处理)
	{
		int sumofNss = 0;                                                  //所有流数相加求和
		std::vector<int> henb_dataToTransmit;                   //所有用户所有流的传输数据大小
		std::vector<std::vector<double> > henb_spectralEfficiency;            //所有用户所有流的传输频谱效率
		for (unsigned int l = 0; l < m_UEs->size(); l++)
		{
			UserEquipment* m_UE = m_UEs->at(l);              //取出每个用户并对每个用户流数Nss进行资源分配

			std::vector<std::vector<int> > ue_cqi = henb->GetUserEquipmentRecord(m_UE->GetIDNetworkNode())->GetCQI();//该用户所有流的的cqi	
			sumofNss = sumofNss + m_UE->GetLtePhy()->Nss;         //所有流数相加求和
			//vector<Application*>* app = m_UE->GetApplist();      //该用户的业务容器

			std::vector<int> ue_dataToTransmit;                    //每个用户所有流的传输数据大小
			std::vector<std::vector<double>> ue_spectralEfficiency;//每个用户所有流的传输频谱效率
			for (int m = 0; m < m_UE->GetLtePhy()->Nss; m++)
			{
				list<Application::flowNode *>* datalist = m_UE->GetApplist()->at(m)->GetDatalist();
				ue_dataToTransmit.push_back(datalist->front()->m_data_size);
				henb_dataToTransmit.push_back(datalist->front()->m_data_size);
				std::vector<double> Nss_sEff;
				for (unsigned int n = 0; n < ue_cqi.at(m).size(); n++)
				{
					double sEff = GetLteMac()->GetAmcModule()->GetEfficiencyFromCQI(ue_cqi.at(m).at(n));
					Nss_sEff.push_back(sEff);
				}
				ue_spectralEfficiency.push_back(Nss_sEff);
				henb_spectralEfficiency.push_back(Nss_sEff);
			}
		}
		//下面进行RB分配    //创建数据流分配优先级矩阵
		int nbOfRBs = GetLtePhy()->GetlteBandwidthManager()->GetDLSubChannels().size();
		std::vector<std::vector<double> >  metrics(nbOfRBs, std::vector<double>(sumofNss));

		for (int i = 0; i < nbOfRBs; i++)
		{
			for (int j = 0; j < sumofNss; j++)
			{
				metrics[i][j] = (henb_spectralEfficiency.at(j).at(i) * 180000.) / henb_dataToTransmit.at(j);
			}
		}

		//具体RB分配开始开始
		for (int s = 0; s < nbOfRBs; s++)
		{
			double targetMetricfirst = 0;
			double targetMetricsecond = 0;
			int k_first = 0;
			int k_second = 0;
			bool RBIsAllocated = false;

			for (int k = 0; k < sumofNss; k++)
			{
				if (metrics[s][k] > targetMetricfirst)
				{
					targetMetricfirst = metrics[s][k];
					k_first = k;
				}
			}
			for (int k = 0; k < sumofNss; k++)
			{
				if (metrics[s][k] > targetMetricsecond && metrics[s][k] < targetMetricfirst)
				{
					targetMetricsecond = metrics[s][k];
					k_second = k;
				}
			}
			RBIsAllocated = true;
			//判定k_first、k_second属于第几个用户第几条数据流
			int k_one, Nss_one, k_two, Nss_two;    //k_one,Nss_one 第k_one个用户的第Nss_one条数据流
			int sum1 = m_UEs->at(0)->GetLtePhy()->Nss;
			int sum2 = m_UEs->at(0)->GetLtePhy()->Nss;
			for (unsigned int i = 0; i < m_UEs->size(); i++)
			{
				if (k_first < m_UEs->at(0)->GetLtePhy()->Nss)
				{
					k_one = 0;
					Nss_one = k_first;
				}
				else if (sum1 <= k_first && k_first< sum1 + m_UEs->at(i + 1)->GetLtePhy()->Nss)
				{
					k_one = i + 1;
					Nss_one = k_first - sum1;
				}
				else
				{
					sum1 = sum1 + m_UEs->at(i + 1)->GetLtePhy()->Nss;
				}
			}
			for (unsigned int i = 0; i < m_UEs->size(); i++)
			{
				if (k_second < m_UEs->at(0)->GetLtePhy()->Nss)
				{
					k_two = 0;
					Nss_two = k_first;
				}
				else if (sum2 <= k_second && k_second < sum2 + m_UEs->at(i + 1)->GetLtePhy()->Nss)
				{
					k_two = i + 1;
					Nss_two = k_second - sum2;
				}
				else
				{
					sum2 = sum2 + m_UEs->at(i + 1)->GetLtePhy()->Nss;
				}
			}
			//给流矩阵赋值
			m_UEs->at(k_one)->GetLtePhy()->m_channelsForRx[Nss_one][s] = 1;
			m_UEs->at(k_two)->GetLtePhy()->m_channelsForRx[Nss_two][s] = 1;
		}
	}
	else
	{
		//send only reference symbols
		//PacketBurst *pb = new PacketBurst ();
		//SendPacketBurst (pb);
	}
	*/
}

vector<PacketBurst*>*
HeNodeB::PacketSchedule(vector<UserEquipment*>* m_UEs)                           //确定给用户所发数据 
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

			int tb_size = ue->GetLteMac()->GetAmcModule()->GetTBSizeFromMCS(m_mcs, ue->GetLtePhy()->m_channelsForRx[0], RBs_FOR_LTE);

			m_SendPacket = m_MacQueue->at(i)->GetPacketToTramsit(m_Net, tb_size);                            //确定此次传输的包  
			ue_packets->AddPacket(m_SendPacket);
		}

		m_SendPackets->push_back(ue_packets);
	}

	return  m_SendPackets;

	//每个henb对应它连接用户应存在一个缓存数据列表，这些数据列表针对的是收到用户的数据请求后建立的，在用户发生切换时，该数据队列会被清空

}

void
HeNodeB::Precoding_Generate(vector<UserEquipment*>* m_UEs)
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
			//用户在该频点的预编码矩阵应为： 发射天线数×该用户在该频点上的总流数
			int rows = Nt;
			int cols = m_UEs->at(j)->GetLtePhy()->num_data_streams[i];
			if (cols>0)
			{
			//cout<< m_UEs->at(j)->GetTargetHenbNode()->GetIDNetworkNode();
				Matrix<complex<double>, Nr, Nt> H_ue = m_UEs->at(j)->GetLteChannel()->channel_Hecoefficients[m_UEs->at(j)->GetTargetHenbNode()->GetIDNetworkNode()][i];  //enb到该用户的信道矩阵Hk（Nr×Nt）
				//SVD分解
			/*	JacobiSVD<MatrixXcf> svd(H_ue, ComputeThinU | ComputeThinV);
				MatrixXcf V = svd.matrixV();
				m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = V;   */   
				// 这里应该是采用相关预编码算法来求预编码，本程序是随机生成
		//		m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = MatrixXcd::Random(rows, cols);
				//---------------------------------------------
				int pmi_index = m_UEs->at(j)->GetTargetHenbNode()->GetUserEquipmentRecord(m_UEs->at(j)->GetIDNetworkNode())->GetPMI().front().at(i);
				switch (cols)
				{
				case 1: {m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W1[pmi_index]; break; }
				case 2: {m_UEs->at(j)->GetLtePhy()->precode_matrix[i] = CodeBook::Init()->W2[pmi_index]; break; }
				default:break;
				}
				GetLtePhy()->precode_matrix[i].middleCols(sum, cols) = m_UEs->at(j)->GetLtePhy()->precode_matrix[i].leftCols(cols);  //通过替换将用户预编码整合成总预编码
				sum += cols;
			}
		}
	}
}

void
HeNodeB::Print()
{
	cout << " HeNodeB object:"
		"\n\t m_idNetworkNode = " << GetIDNetworkNode() <<
		"\n\t Served Users: " <<
		endl;

	vector<UserEquipmentRecord*>* records = GetUserEquipmentRecords();
	UserEquipmentRecord *record;
	vector<UserEquipmentRecord*>::iterator iter;
	for (iter = records->begin(); iter != records->end(); iter++)
	{
		record = *iter;
		cout << "\t\t idUE = " << record->GetUE()->
			GetIDNetworkNode() << endl;
	}
}
