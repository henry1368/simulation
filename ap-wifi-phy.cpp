#include "ap-wifi-phy.h"
#include "wifi-phy.h"
#include "NetworkNode.h"
#include "wifi-bandwidth-manager.h"
#include "UserEquipment.h"

#include "packet-burst.h"
#include "AP.h"
#include "Simulator.h"
#include "parameter.h"
#include "wifi-phy.h"
#include "DcfState_Cluster.h"
#include "dcf-manager.h"
#include "WIFI_channel.h"
//#include "math.h"


/*
* Noise is computed as follows:
*  - noise figure = 2.5
*  - n0 = -174 dBm
*  - sub channel bandwidth = 180 kHz
*
*  noise_db = noise figure + n0 + 10log10 (180000) - 30 = -148.95
*/
#define NOISE -148.95
using namespace std;

APWifiPhy::APWifiPhy()
{
	SetDevice(NULL);
	SetwifiBandwidthManager(NULL);
}

APWifiPhy::~APWifiPhy()
{
	Destroy();
}



void
APWifiPhy::StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets)            
{

	TxPowerAllocation(Dst_UEs);                    //进行功率配置，信号配置等

//	std::cout << "AP " << GetDevice()->GetIDNetworkNode() << " 开始物理层传输" << std::endl;
	int m = 0;
	double time = 5844;      //传输时长为2332us
	std::vector<UserEquipment*>::iterator it;
	for (it = Dst_UEs->begin(); it != Dst_UEs->end(); it++)
	{
		PacketBurst* P = Tx_Packets->at(m);   //给用户的包
		double L = P->GetSize();
		Simulator::Init()->Schedule("wifi", time, &UeWifiPhy::StartRx, (*it)->GetWifiPhy(), P); //将对应的接收事件入队
		m++;
	}
  

	Simulator::Init()->Schedule("wifi", time+60, &DcfState_Cluster::update_cluster_info, DcfManager::Init()->GetCluster((AP*)GetDevice()));//更新位置
}




void 
APWifiPhy::TxPowerAllocation(vector<UserEquipment*>* Dst_UEs)        //功率配置，确定每个资源块上所有用户的流功率分配矩阵
{	            
	if (WIFI_BANDWIDTH == 80)
		SetTxPower(41);
	else if (WIFI_BANDWIDTH == 160)
		SetTxPower(44);

	double totalpower = pow(10., GetTxPower() / 10) / 1000;             //自然单位

	double averpower = totalpower / RBs_FOR_WIFI; //每个资源块能量 dB

	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		double  d;

		int N = m_num_tx_streams[i];                     //该频点上的所有的流数

		if (N > 0)
		{
			allNss_power[i] = MatrixXd::Zero(N, N);         //功率分配矩阵初始化

			d = averpower / N;                      //目前是每频点上所有用户流功率均分
			int sum = 0;
			for (unsigned int j = 0; j < Dst_UEs->size(); j++)
			{
				UserEquipment* ue = Dst_UEs->at(j);
				int n = ue->GetWifiPhy()->num_data_streams[i];                        //该用户在该频点上的流数
				if (n>0)
				{
					//double P_loss = ue->GetWifiChannel()->AP_power_loss[GetDevice()->GetIDNetworkNode()];
					//P_loss = pow(10, P_loss / 10);
					//ue->GetWifiPhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d)*sqrt(P_loss);        //该用户在该频点上的流功率分配矩阵：根号（P_tx*P_loss）

					ue->GetWifiPhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d);
					for (int k = 0; k < n; k++)
						allNss_power[i](sum + k, sum + k) = sqrt(d);     //在该频点上的的总发射功率矩阵

					sum += n;
				}
			}
		}
	}
	 		
}

