#include "enb-lte-phy.h"
#include "lte-phy.h"
#include "NetworkNode.h"
#include "lte-bandwidth-manager.h"
#include "packet-burst.h"
#include "ENodeB.h"
#include "UserEquipment.h"
#include "simulator.h"
#include "LTE-A_channel.h"
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

EnbLtePhy::EnbLtePhy()
{
	SetDevice(NULL);
    SetlteBandwidthManager(NULL);
}

EnbLtePhy::~EnbLtePhy()
{
	Destroy();
}


void
EnbLtePhy::StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets)            //这里还需要进一步进行修改
{
	TxPowerAllocation( Dst_UEs);                    //进行功率配置，信号配置等

	std::cout << "ENodeB " << GetDevice()->GetIDNetworkNode() << " 开始物理层传输" << std::endl;
	int m = 0;
	std::vector<UserEquipment*>::iterator it;
	for (it = Dst_UEs->begin(); it != Dst_UEs->end(); it++)
	{
		Simulator::Init()->Schedule("lte", 1000, &UeLtePhy::StartRx,(*it)->GetLtePhy(), Tx_Packets->at(m)); //将对应的接收事件入队
		m++;
	}
}

void
EnbLtePhy::TxPowerAllocation( vector<UserEquipment*>* Dst_UEs)        //功率配置，确定每个资源块上所有用户的流功率分配矩阵
{
	if (LTE_BANDWIDTH == 10)
		SetTxPower(46);
	else if (LTE_BANDWIDTH == 20)
		SetTxPower(49);

	double totalpower = pow(10., GetTxPower() / 10)/1000;                         //自然单位

	double averpower = totalpower / RBs_FOR_LTE; //每个资源块能量 dB

	for (int i = 0; i < RBs_FOR_LTE; i++)
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
			int n = ue->GetLtePhy()->num_data_streams[i];                        //该用户在该频点上的流数
			if (n>0)
			{
				//double P_loss = ue->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]; 
				//P_loss = pow(10, P_loss / 10);
    //            ue->GetLtePhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d)*sqrt(P_loss);        //该用户在该频点上的流功率分配矩阵：根号（P_tx*P_loss）

				ue->GetLtePhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d);

				for (int k = 0; k < n; k++)
					allNss_power[i](sum + k, sum + k) = sqrt(d);     //在该频点上的的总发射功率矩阵

				sum += n;
			}			
		  }
		}


		/*
		cout << N << endl;
		cout << d << endl;
		allNss_power[i] = MatrixXd::Identity(N, N);
		cout << allNss_power[i] << endl << endl;
		allNss_power[i] *= sqrt(d)*sqrt(Dst_UEs->at(0)->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]);  //所有用户的流功率分配矩阵
		cout << Dst_UEs->at(0)->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]<<endl;
		cout << allNss_power[i] << endl << endl;
		*/
	}

}