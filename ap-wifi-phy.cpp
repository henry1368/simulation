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

	TxPowerAllocation(Dst_UEs);                    //���й������ã��ź����õ�

//	std::cout << "AP " << GetDevice()->GetIDNetworkNode() << " ��ʼ����㴫��" << std::endl;
	int m = 0;
	double time = 5844;      //����ʱ��Ϊ2332us
	std::vector<UserEquipment*>::iterator it;
	for (it = Dst_UEs->begin(); it != Dst_UEs->end(); it++)
	{
		PacketBurst* P = Tx_Packets->at(m);   //���û��İ�
		double L = P->GetSize();
		Simulator::Init()->Schedule("wifi", time, &UeWifiPhy::StartRx, (*it)->GetWifiPhy(), P); //����Ӧ�Ľ����¼����
		m++;
	}
  

	Simulator::Init()->Schedule("wifi", time+60, &DcfState_Cluster::update_cluster_info, DcfManager::Init()->GetCluster((AP*)GetDevice()));//����λ��
}




void 
APWifiPhy::TxPowerAllocation(vector<UserEquipment*>* Dst_UEs)        //�������ã�ȷ��ÿ����Դ���������û��������ʷ������
{	            
	if (WIFI_BANDWIDTH == 80)
		SetTxPower(41);
	else if (WIFI_BANDWIDTH == 160)
		SetTxPower(44);

	double totalpower = pow(10., GetTxPower() / 10) / 1000;             //��Ȼ��λ

	double averpower = totalpower / RBs_FOR_WIFI; //ÿ����Դ������ dB

	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		double  d;

		int N = m_num_tx_streams[i];                     //��Ƶ���ϵ����е�����

		if (N > 0)
		{
			allNss_power[i] = MatrixXd::Zero(N, N);         //���ʷ�������ʼ��

			d = averpower / N;                      //Ŀǰ��ÿƵ���������û������ʾ���
			int sum = 0;
			for (unsigned int j = 0; j < Dst_UEs->size(); j++)
			{
				UserEquipment* ue = Dst_UEs->at(j);
				int n = ue->GetWifiPhy()->num_data_streams[i];                        //���û��ڸ�Ƶ���ϵ�����
				if (n>0)
				{
					//double P_loss = ue->GetWifiChannel()->AP_power_loss[GetDevice()->GetIDNetworkNode()];
					//P_loss = pow(10, P_loss / 10);
					//ue->GetWifiPhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d)*sqrt(P_loss);        //���û��ڸ�Ƶ���ϵ������ʷ�����󣺸��ţ�P_tx*P_loss��

					ue->GetWifiPhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d);
					for (int k = 0; k < n; k++)
						allNss_power[i](sum + k, sum + k) = sqrt(d);     //�ڸ�Ƶ���ϵĵ��ܷ��书�ʾ���

					sum += n;
				}
			}
		}
	}
	 		
}

