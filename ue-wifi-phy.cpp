
#include "ue-wifi-phy.h"
#include "NetworkManager.h"
#include "dcf-manager.h"
#include "DcfState_Cluster.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "AP.h"
#include "ap-mac.h"
#include "wifi-bandwidth-manager.h"
#include "interference.h"
#include "CodeBook.h"
#include "ap-wifi-phy.h"
#include "Packet.h"
#include "packet-burst.h"
#include "WIFI_channel.h"
#include "parameter.h"
#include "MacQueue.h"
#include "EffectiveSINR.h"
#include "BLERCurves.h"
#include "Fountain.h"
#include "simulator.h"
#include "application.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

/*

#include "wifi-cqimanager.h"

*/

/*
* Noise is computed as follows:
*  - noise figure = 2.5
*  - n0 = -174 dBm
*  - sub channel bandwidth = 180 kHz
*
*  noise_db = noise figure + n0 + 10log10 (180000) - 30 = -148.95
*/
#define NOISE -148.95

UeWifiPhy::UeWifiPhy()
{

	SetDevice(NULL);
	SetwifiBandwidthManager(NULL);
}

UeWifiPhy::~UeWifiPhy()
{
	Destroy();
}



void
UeWifiPhy::StartRx(PacketBurst* p)           //一个用户接收数据包（可以是跨业务或者单业务跨不同数据块，目前是单业务单数据块）
{

	UserEquipment* ue = (UserEquipment*)GetDevice();

	DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_RxStart = Simulator::Init()->Now();  //记录开始接收时间
	
	cout << "用户" << GetDevice()->GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << " 开始接收来自AP" << ue->GetTargetAPNode()->GetIDNetworkNode()<<"的WIFI数据包:"<<std::endl;

	double* m_Sinr = SINR_measure();               //MMSE算出等效SINR
	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		if (p->GetPackets().at(0)->m_retransmit_number>0)            //重传次数不为0,则该包的SINR计算需要结合上次的SINR记录              
			m_measuredSinr[i] += m_Sinr[i];                          //如果是重传则与上次的SINR合并得出新的SINR
		else
			m_measuredSinr[i] = m_Sinr[i];                           //不是重传则记录本次的计算结果
	}

	delete  m_Sinr;



	int m_mcsIndexForRx = ue->GetTargetAPNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs();
	double m_eesmSINR = EffectiveSINR::WIFI_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);
	double m_bler = BLERCurves::WIFI_getBLER(m_eesmSINR, m_mcsIndexForRx, Nss);             //求出最终误包率

//	m_bler = 0.001;
	double   random_value = ((double)rand()) / 0x7fff;           //产生随机数,判断此次传输是否成功 


	for (uint32_t i=0; i < p->GetNPackets();i++)
	{

		Packet * m_packet = p->GetPackets().at(i);      //取出当前所收的包
		int n = m_packet->m_num_application;            //n为对应用户的业务索引
		int m;                                         //m为对应该业务的缓存队列
		int t = 0;
		std::vector<MacQueue*>* m_macqueue = ue->GetTargetAPNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		vector<MacQueue*>::iterator it1;
		for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
		{
			if ((*it1)->GetAppNum() == n)
			{
				m = t;
			}
			t++;                                        //m为AP对应的缓存队列索引
		}


		if (random_value <= m_bler)                     //失败
		{

			//分为喷泉码的包和非喷泉码的包进行处理

			if (m_packet->m_is_FountainCode)
			{
				// step1： 判断该包所在的业务数据已通过另一网络满足
				int index = m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_index; //取出对应的索引
				int *ptr = ue->GetApplist()->at(n)->m_flag_ptr;
				if (ptr[index] == 1)                                         //对应的标志位,表示该数据已被满足
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();        //清除缓存,保证队列中有数据
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

				}
				else
				{
					cout << "收包失败" << endl;
//					m_packet->m_retransmit_number++;
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
				}
			}
			else
			{
				cout << "收包失败" << endl;
//				m_packet->m_retransmit_number++;
				/*
				if(m_packet->m_retransmit_number>=重传上限)
				{
				丢弃该包：去掉该次的业务需求
				}
				*/
//				m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
			}
		}

		else                                     //成功       
		{
			cout << "收包成功,";
			if (m_packet->m_is_FountainCode)                             //喷泉码包
			{
				// step1： 判断该包所在的业务数据已通过另一网络满足
				int index = m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_index; //取出对应的索引
				int *ptr = ue->GetApplist()->at(n)->m_flag_ptr;
				long double *PTR = ue->GetApplist()->at(n)->m_Fountain_data;
				if (ptr[index] == 1)                                         //对应的标志位,表示该数据已被满足
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();        //清除缓存,保证队列中有数据
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

				}
				else
				{
					PTR[index] = PTR[index] + m_packet->m_size;     //收到数据的总量
					double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //数据分为2000个喷泉码小包，求出喷泉码包长
					int Numofpak =int( PTR[index] / pak_length);                   //收到喷泉码包的个数 =总量/喷泉码包长

					cout << "收到的数据量为:" << m_packet->m_size << ",总数据量为" << PTR[index] << endl;// "bit,喷泉码包个数:" << Numofpak << endl;
					//这一步根据曲线读出接包概率
					double pob = Fountain_decode[Numofpak];
					double Rm = ((double)rand()) / 0x7fff;
					if (Rm <= pob)
					{
						//解包成功
	//					cout << "解包成功" << endl;
						ptr[index] = 1;   //设置用户的接收信息
						m_macqueue->at(m)->GetPacketQueue()->pop_front();          //清除缓存,保证队列中有数据
						if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
							m_macqueue->erase(m_macqueue->begin() + m);

						ue->GetApplist()->at(n)->GetDatalist()->pop_front();        //清除用户的数据请求，保证队列中有数据
						if (ue->GetApplist()->at(n)->GetDatalist()->size() == 0)
							ue->GetApplist()->erase(ue->GetApplist()->begin() + n);
					}
				}
			}

			else                  //非喷泉码
			{
				if (m_packet->m_isAFragment == false || m_packet->m_isTheLatestFragment == true)    //该包代表此次数据的需求完成
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();          //清除缓存,保证队列中有数据
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

					ue->GetApplist()->at(n)->GetDatalist()->pop_front();        //清除用户的数据请求，保证队列中有数据
					if (ue->GetApplist()->at(n)->GetDatalist()->size() == 0)
						ue->GetApplist()->erase(ue->GetApplist()->begin() + n);

				}
				else
				{
					cout << "此包为普通分片（但不是最后一个）" << endl;
					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = NULL;
				}
			}
       
          //在次数存储接收数据
			int index = int(Simulator::Init()->Now() / 1000);                     //量化到第几个TTI

			ue->WIFI_Receive_Data[index] = ue->WIFI_Receive_Data[index]+ m_packet->m_size;  //数据增加
		}
	}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);           //计算反馈

		DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_RxEnd = Simulator::Init()->Now() + 44;   //记录所在簇接收结束时间

		DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_lastNavDuration = DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_lastNavStart+224;  //记录所在簇NAV时长

		ue->GetTargetAPNode()->GetApMac()->GetDcfstate()->ResetState();   //重新设置AP退避状态

}




double *
UeWifiPhy::SINR_measure()
{
	UserEquipment*ue = (UserEquipment*)GetDevice(); //所在用户
	AP* m_ap = ue->GetTargetAPNode();               //所连ap

	double* eff_Sinr = new double[RBs_FOR_WIFI];
	double** m_Sinr = new double*[Nss];
	for (int i = 0; i < Nss; i++)
	{
		m_Sinr[i] = new double[RBs_FOR_WIFI];         
	}

	Matrix<complex<double>, Nr, Nr>  H_ap_interference[RBs_FOR_WIFI]; 
	
	GetInterference()->WIFI_ComputeInterference(ue, H_ap_interference);    //计算其它AP在该每个频点上的干扰矩阵（Nr×Nr）

	   
   double sigma2 = pow(10, (-174 + 7 + 10 * log10(312.5 * 1000) ) / 10)/1000;            //噪声功率

   for (int i = 0; i < RBs_FOR_WIFI; i++)
   {
	   if (num_data_streams[i] == 0)
	   {
		   for (int k = 0; k < Nss; k++)
			   m_Sinr[k][i] = 0;				      //每个流上的SINR
	   }
	   else
	   {
		   Matrix<complex<double>, Nr, Nt> H_ue = ue->GetWifiChannel()->channel_APcoefficients[m_ap->GetIDNetworkNode()][i];  //ap到该用户的信道矩阵Hk（Nr×Nt）

		   MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (接收天线数×该用户流数)
		   H_hat = H_ue*precode_matrix[i] * Nss_power[i];

		   MatrixXcd H_all = MatrixXcd::Identity(Nr, m_ap->GetWifiPhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (接收天线数×该用户流数)
		   H_all = H_ue* m_ap->GetWifiPhy()->precode_matrix[i] * m_ap->GetWifiPhy()->allNss_power[i];  //Hk*W*P

		   Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //多用户干扰(Nr×Nr)     （单用户时，此矩阵应为零矩阵）

		   Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //噪声方阵

		   MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE算子

		   L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_ap_interference[i]).inverse()); //(用户流数×发射天线数)

		   //计算SINR:

		   MatrixXcd D = MatrixXcd::Zero(num_data_streams[i], num_data_streams[i]);
		   MatrixXcd diag_D = MatrixXcd::Zero(num_data_streams[i], num_data_streams[i]);
		   D = L_mmse*H_hat;
		   diag_D = D.diagonal().asDiagonal();

		   VectorXcd P_ue = VectorXcd::Zero(num_data_streams[i], 1);
		   P_ue = (diag_D*(diag_D.adjoint())).diagonal();          //有用信号功率

		   VectorXcd P_inter_stream = VectorXcd::Zero(num_data_streams[i], 1);
		   P_inter_stream = (((D - diag_D)*(D - diag_D)).adjoint()).diagonal();  //流间干扰

		   VectorXcd P_inter_ue = VectorXcd::Zero(num_data_streams[i], 1);
	       P_inter_ue = (L_mmse*H_mu_in*((L_mmse*H_mu_in).adjoint())).diagonal(); //多用户干扰

		   VectorXcd P_noise = VectorXcd::Zero(num_data_streams[i], 1);
		   P_noise = ((L_mmse*(L_mmse.adjoint())).diagonal())*sigma2;               //噪声功率

		   VectorXcd P_ap = VectorXcd::Zero(num_data_streams[i], 1);
//		   P_ap = ((L_mmse*H_ap_interference[i])*((L_mmse*H_ap_interference[i]).adjoint())).diagonal(); //其它AP干扰       //这里有错   

		   P_ap = ((L_mmse*H_ap_interference[i])*(L_mmse.adjoint())).diagonal(); //其它AP干扰       //这里有错
		   VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
		   Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_ap).real().array();    //这里对角线上的元素是实值，由于矩阵属性为复，故求出取实部

		   int M = 0;
		   for (int N = 0; N < Nss; N++)         //赋值
		   {
			   if (m_channelsForRx[N][i]>0)
			   {
				   m_Sinr[N][i] = Sinr(M);         
				   M++;
			   }	  
		  }


	   }
   }

	//计算出每个流上的SINR后，在这里进行等效
	for (int k = 0; k < RBs_FOR_WIFI; k++)
	{
		eff_Sinr[k] = m_Sinr[0][k];                //注：现在用户均为单流，故无等效
	}

	//删除内存，返回
	for (int i = 0; i < Nss; i++)
		delete[] m_Sinr[i];
	delete[]  m_Sinr;
	return eff_Sinr;

	
}



/*

	//COMPUTE THE SINR
	std::vector<double> rxSignalValues;
	std::vector<double>::iterator it;

	rxSignalValues = txSignal->Getvalues();

	//compute noise + interference
	double interference;
	if (GetInterference() != NULL)
	{
	interference = GetInterference()->ComputeInterference((UserEquipment*)GetDevice());
	}
	else
	{
	interference = 0;
	}

	double noise_interference = 10. * log10(pow(10., NOISE / 10) + interference); // dB


	for (it = rxSignalValues.begin(); it != rxSignalValues.end(); it++)
	{
	double power; // power transmission for the current sub channel [dB]
	if ((*it) != 0.)
	{
	power = (*it);
	}
	else
	{
	power = 0.;
	}
	m_measuredSinr.push_back(power - noise_interference);
	}

	//CHECK FOR PHY ERROR

	bool phyError;

	if (GetErrorModel() != NULL && m_channelsForRx.size() > 0)
	{
	std::vector<int> cqi_;
	for (int i = 0; i < m_mcsIndexForRx.size(); i++)
	{
	AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
	int cqi = amc->GetCQIFromMCS(m_mcsIndexForRx.at(i));
	cqi_.push_back(cqi);
	}
	phyError = GetErrorModel()->CheckForPhysicalError(m_channelsForRx, cqi_, m_measuredSinr);

	if (_PHY_TRACING_)
	{
	if (phyError)
	{
	std::cout << "**** YES PHY ERROR (node " << GetDevice()->GetIDNetworkNode() << ") ****" << std::endl;
	}
	else
	{
	std::cout << "**** NO PHY ERROR (node " << GetDevice()->GetIDNetworkNode() << ") ****" << std::endl;
	}
	}
	}
	else
	{
	phyError = false;
	}


	#ifdef PLOT_SINR_3D
	double effective_sinr = GetEesmEffectiveSinr(m_measuredSinr);
	if (effective_sinr > 40) effective_sinr = 40;
	int cqi = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effective_sinr);
	int MCS_ = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetMCSFromCQI(cqi);
	int TBS_ = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(MCS_, GetBandwidthManager()->GetDlSubChannels().size());
	std::cout << "SINR " << GetDevice()->GetIDNetworkNode() << " " <<
	GetDevice()->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX() << " " <<
	GetDevice()->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY() << " " <<
	effective_sinr << " " <<
	MCS_ << " " << TBS_ << std::endl;
	#endif

	if (!phyError && p->GetNPackets() > 0)
	{
	//FORWARD RECEIVED PACKETS TO THE DEVICE
	GetDevice()->ReceivePacketBurst(p);
	}



	//CQI report
	CreateCqiFeedbacks(m_measuredSinr);

	m_channelsForRx.clear();
	m_channelsForTx.clear();
	m_mcsIndexForRx.clear();
	m_mcsIndexForTx.clear();

	delete txSignal;
	delete p;

	*/

void
UeWifiPhy::CreateAndSendFeedbacks(double sinr[], double m_eesmSINR)           //计算反馈
{
	vector<int> m_cqi = CreateCqiFeedbacks(m_eesmSINR);
	vector<int> m_pmi = CreatePMIFeedbacks(sinr);
	SendFeedbacks(m_cqi, m_pmi);
}

vector<int>
UeWifiPhy::CreateCqiFeedbacks(double m_eesmSINR)
{
	double WIFI_SINRForCQIIndex[10] = { -2.25, 2, 4.5, 8, 11.5, 15.75, 17.5, 18.25, 23.5, 24.25 };//添加映射表
	int CQIIndex;
	for (int k = 0; k < 9; k++)
	{
		if (WIFI_SINRForCQIIndex[k] <= m_eesmSINR && m_eesmSINR < WIFI_SINRForCQIIndex[k + 1])
		{
			 CQIIndex = k;
			 break;
		}
	}

	if (WIFI_SINRForCQIIndex[0] > m_eesmSINR)
		    CQIIndex = 0;
	else if (WIFI_SINRForCQIIndex[9] < m_eesmSINR)
		    CQIIndex = 9;

//以上确定宽带CQI
	vector<int> cqi;
	for (int i = 0; i <RBs_FOR_WIFI; i++)   		
		cqi.push_back(CQIIndex);
	return cqi;
}

vector<int>
UeWifiPhy::CreatePMIFeedbacks(double sinr[])
{
	UserEquipment*ue = (UserEquipment*)GetDevice();
	vector<int> pmi;

	for (int i = 0; i < RBs_FOR_WIFI; i++)
   {
		Matrix<complex<double>, Nr, Nt> H_ue = ue->GetWifiChannel()->channel_APcoefficients[ue->GetTargetAPNode()->GetIDNetworkNode()][i];
		int M = ue->GetLtePhy()->num_data_streams[i];
		double targetcapacity = 0;
		complex<double> detcapacity;
		double capacity;
		int pmi_;
		switch (M)
		{
		  case 1:
			{
					  MatrixXcd I = MatrixXcd::Identity(1, 1);
					  for (int j = 0; j < 16; j++)
					  {
						  MatrixXcd X = MatrixXcd::Zero(1, 1);
						  X = ((CodeBook::Init()->W1[j]).transpose()).conjugate()*(H_ue.transpose()).conjugate()*H_ue*(CodeBook::Init()->W1[j]);
						  detcapacity = (I + X*sinr[i]).determinant();
						  capacity = sqrt(detcapacity.real()*detcapacity.real() + detcapacity.imag()*detcapacity.imag());
						  if (capacity > targetcapacity)
						  {
							  targetcapacity = capacity;
							  pmi_ = j;
						  }
					  }
					  break;
			}
			case 2:
			{
					  MatrixXcd I = MatrixXcd::Identity(2, 2);
					  for (int j = 0; j < 16; j++)
					  {
						  MatrixXcd X = MatrixXcd::Zero(2, 2);
						  X = ((CodeBook::Init()->W2[j]).transpose()).conjugate()*(H_ue.transpose()).conjugate()*H_ue*(CodeBook::Init()->W2[j]);
						  detcapacity = (I + X*sinr[i] / 2).determinant();
						  capacity = sqrt(detcapacity.real()*detcapacity.real() + detcapacity.imag()*detcapacity.imag());
						  if (capacity > targetcapacity)
						  {
							  targetcapacity = capacity;
							  pmi_ = j;
						  }
					  }
					  break;
			}
			default:break;
			}
			pmi.push_back(pmi_);
		}

	
	return pmi;
}

void
UeWifiPhy::SendFeedbacks(vector<int> cqi, vector<int> pmi)
{
	UserEquipment*ue = (UserEquipment*)GetDevice();

	AP::UserEquipmentRecord* Ptr = ue->GetTargetAPNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode());


	//Ptr->GetCQI().at(0).clear();
	//Ptr->GetCQI().erase(Ptr->GetCQI().begin());  
	//Ptr->GetCQI().push_back(cqi);                          //缓存cqi

	vector<vector<int>> X = Ptr->GetCQI();
	X.at(0).clear();
	X.erase(X.begin());
	X.push_back(cqi);
	Ptr->SetCQI(X);



	//Ptr->GetPMI().at(0).clear();
	//Ptr->GetPMI().erase(Ptr->GetPMI().begin());           //缓存pmi
	//Ptr->GetPMI().push_back(pmi);

	vector<vector<int>> Y = Ptr->GetPMI();
	Y.at(0).clear();
	Y.erase(Y.begin());
	Y.push_back(cqi);
	Ptr->SetPMI(Y);


}

/*
void
UeWifiPhy::SendIdealControlMessage(IdealControlMessage *msg)
{
#ifdef TEST_CQI_FEEDBACKS
std::cout << "SendIdealControlMessage (PHY) from  " << msg->GetSourceDevice()->GetIDNetworkNode()
<< " to " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
#endif

NetworkNode* dst = msg->GetDestinationDevice();
dst->GetWifiPhy()->ReceiveIdealControlMessage(msg);

delete msg;
}

void
UeWifiPhy::ReceiveIdealControlMessage(IdealControlMessage *msg)
{
if (msg->GetMessageType() == IdealControlMessage::ALLOCATION_MAP)
{
//std::cout << "ReceiveIdealControlMessage, node " << GetDevice()->GetIDNetworkNode() << std::endl;

m_channelsForRx.clear();
m_channelsForTx.clear();
m_mcsIndexForRx.clear();
m_mcsIndexForTx.clear();

PdcchMapIdealControlMessage *map = (PdcchMapIdealControlMessage*)msg;
PdcchMapIdealControlMessage::IdealPdcchMessage *map2 = map->GetMessage();
PdcchMapIdealControlMessage::IdealPdcchMessage::iterator it;

int node = GetDevice()->GetIDNetworkNode();

for (it = map2->begin(); it != map2->end(); it++)
{
if ((*it).m_ue->GetIDNetworkNode() == node)
{
if ((*it).m_direction == PdcchMapIdealControlMessage::DOWNLINK)
{
//std::cout << "\t channel " << (*it).m_idSubChannel
//		  << " mcs "<< (*it).m_mcsIndex << std::endl;

m_channelsForRx.push_back((*it).m_idSubChannel);
m_mcsIndexForRx.push_back((*it).m_mcsIndex);
}
else if ((*it).m_direction == PdcchMapIdealControlMessage::UPLINK)
{
m_channelsForTx.push_back((*it).m_idSubChannel);
m_mcsIndexForTx.push_back((*it).m_mcsIndex);
}
}
}

if (m_channelsForTx.size() > 0)
{
DoSetBandwidthManager();
UeMacEntity* mac = (UeMacEntity*)GetDevice()->GetProtocolStack()->GetMacEntity();
mac->ScheduleUplinkTransmission(m_channelsForTx.size(), m_mcsIndexForTx.at(0));
}
}
}
*/
