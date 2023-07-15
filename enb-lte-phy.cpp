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
EnbLtePhy::StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets)            //���ﻹ��Ҫ��һ�������޸�
{
	TxPowerAllocation( Dst_UEs);                    //���й������ã��ź����õ�

	std::cout << "ENodeB " << GetDevice()->GetIDNetworkNode() << " ��ʼ����㴫��" << std::endl;
	int m = 0;
	std::vector<UserEquipment*>::iterator it;
	for (it = Dst_UEs->begin(); it != Dst_UEs->end(); it++)
	{
		Simulator::Init()->Schedule("lte", 1000, &UeLtePhy::StartRx,(*it)->GetLtePhy(), Tx_Packets->at(m)); //����Ӧ�Ľ����¼����
		m++;
	}
}

void
EnbLtePhy::TxPowerAllocation( vector<UserEquipment*>* Dst_UEs)        //�������ã�ȷ��ÿ����Դ���������û��������ʷ������
{
	if (LTE_BANDWIDTH == 10)
		SetTxPower(46);
	else if (LTE_BANDWIDTH == 20)
		SetTxPower(49);

	double totalpower = pow(10., GetTxPower() / 10)/1000;                         //��Ȼ��λ

	double averpower = totalpower / RBs_FOR_LTE; //ÿ����Դ������ dB

	for (int i = 0; i < RBs_FOR_LTE; i++)
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
			int n = ue->GetLtePhy()->num_data_streams[i];                        //���û��ڸ�Ƶ���ϵ�����
			if (n>0)
			{
				//double P_loss = ue->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]; 
				//P_loss = pow(10, P_loss / 10);
    //            ue->GetLtePhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d)*sqrt(P_loss);        //���û��ڸ�Ƶ���ϵ������ʷ�����󣺸��ţ�P_tx*P_loss��

				ue->GetLtePhy()->Nss_power[i] = MatrixXd::Identity(n, n)*sqrt(d);

				for (int k = 0; k < n; k++)
					allNss_power[i](sum + k, sum + k) = sqrt(d);     //�ڸ�Ƶ���ϵĵ��ܷ��书�ʾ���

				sum += n;
			}			
		  }
		}


		/*
		cout << N << endl;
		cout << d << endl;
		allNss_power[i] = MatrixXd::Identity(N, N);
		cout << allNss_power[i] << endl << endl;
		allNss_power[i] *= sqrt(d)*sqrt(Dst_UEs->at(0)->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]);  //�����û��������ʷ������
		cout << Dst_UEs->at(0)->GetLteChannel()->eNodeB_power_loss[GetDevice()->GetIDNetworkNode()]<<endl;
		cout << allNss_power[i] << endl << endl;
		*/
	}

}