#include "simulator.h"
#include "interference.h"
#include "NetworkManager.h"
#include  "dcf-manager.h"
#include  "DcfState_Cluster.h"
#include "UserEquipment.h"
#include "AP.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "enb-lte-phy.h"
#include  "henb-lte-phy.h"
#include "ap-wifi-phy.h"
#include "wifi-bandwidth-manager.h"
#include "lte-bandwidth-manager.h"
#include "WIFI_channel.h"
#include "LTE-A_channel.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

Interference::Interference()
{}

Interference::~Interference()
{}


void
Interference::WIFI_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>*AP_interference_matrix)
{
	/*
	WIFI�в�ͬ��AP���ò�ͬ���ŵ��������Ƶ��ѡ����ڲ��죬��ͬ��AP����ܴ����Ų��ֵ�Ƶ�ʽ�����
	����ڼ������ʱ����Ҫ�ҳ�����AP������Ƶ�ʸ��ã�������ʱ�䲻ͬ��������ǰʱ�̴��ڱ��wifi�� 
	�ķ�������ʱ��������ʱ�Ŵ��ڸ��š�
	*/

	//��ʼ������Ƶ���ϵ�����AP����
	for (int i = 0; i<RBs_FOR_WIFI; i++)
		AP_interference_matrix[i] = MatrixXcd::Zero(Nr, Nr);

	double time_now = Simulator::Init()->Now();                       //��ǰʱ��
	vector<AP*> *m_aps = NetworkManager::Init()->GetAPContainer();    //����ap
	for (int k = 0; k < RBs_FOR_WIFI; k++)
		{
			double frequence = ue->GetWifiPhy()->GetwifiBandwidthManager()->GetDLSubChannels().at(k);   //Ƶ��
			Matrix<complex<double>, Nr, Nr> tn_matrix = AP_interference_matrix[k];                       //��ʼ��Ϊ�����

			for (int t = 0; t < nb_apingroup-1; t++)
			{
				AP* ap = m_aps->at(ue->ID_AP[t + 1]);                              //ͬȺ�ڵ�����AP��
				DcfState_Cluster *L = DcfManager::Init()->GetCluster(ap);
				if (L->m_RxStart <= time_now && L->m_RxEnd >= time_now)            //�����¼��غ��Ҵ���Ƶ�ʸ���
				{
					for (int m = 0; m < RBs_FOR_WIFI; m++)
					{
						if (ap->GetWifiPhy()->GetwifiBandwidthManager()->GetDLSubChannels().at(m) == frequence && ap->GetWifiPhy()->m_num_tx_streams[m]>0)  //�ҵ���ӦƵ��
						{
							double P_loss = ue->GetWifiChannel()->AP_power_loss[ap->GetIDNetworkNode()];
							P_loss = pow(10, P_loss / 10);
							Matrix<complex<double>, Nr, Nt> H_ue = ue->GetWifiChannel()->channel_APcoefficients[ap->GetIDNetworkNode()][m];           //��AP���û����ŵ�����
							tn_matrix += H_ue*(ap->GetWifiPhy()->precode_matrix[m])*(ap->GetWifiPhy()->allNss_power[m])*(ap->GetWifiPhy()->allNss_power[m]).adjoint()*(ap->GetWifiPhy()->precode_matrix[m]).adjoint()*H_ue.adjoint()*P_loss;
							break;                           //�������������� (Hki��Wi*Pi)*(Hki��Wi*Pi).adjoint �ۼ�
						}
					}
				}
			}
			AP_interference_matrix[k] = tn_matrix;
		}
   
}






void
Interference::LTE_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>* Enb_interference_matrix)      //�����������վ��С��վ����
{
	/*
	����������С��վ������ȫƵ��
	*/

	for (int i = 0; i< RBs_FOR_LTE; i++)
		Enb_interference_matrix[i] = MatrixXcd::Zero(Nr, Nr);


	vector<ENodeB*> *m_enbs = NetworkManager::Init()->GetENodeBContainer();       //���к��վ
	vector<HeNodeB*> *m_henbs = NetworkManager::Init()->GetHeNodeBContainer();    //����С��վ

	for (int k = 0; k < RBs_FOR_LTE; k++)
	{

		Matrix<complex<double>, Nr, Nr> tn_matrix = Enb_interference_matrix[k];                        //��ʼ��Ϊ�����
		vector<ENodeB*>::iterator it;
		vector<HeNodeB*>::iterator it1;

		if (ue->GetTargetEnbNode() != NULL)
		{
			for (int i = 0; i < min(Num_Enb_Select, nb_totalEnb-1); i++)
			{
				ENodeB* enb = m_enbs->at(ue->ID_Enb[i+1]);
				if (enb->GetLtePhy()->m_num_tx_streams[k]>0)           //���ڸ���
				{
					Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Ecoefficients[enb->GetIDNetworkNode()][k];           //�ú��վ���û����ŵ�����
					/*
					cout << H_ue << endl << endl;
					cout << enb->GetLtePhy()->precode_matrix[k] << endl << endl;
					cout << enb->GetLtePhy()->allNss_power[k]<<endl<<endl;                 //�д�
					cout << (enb->GetLtePhy()->allNss_power[k]).adjoint() << endl << endl; //�д�
					cout << (enb->GetLtePhy()->precode_matrix[k]).adjoint() << endl << endl;
					cout << H_ue.adjoint() << endl << endl;
					*/

					double P_loss = ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]; 
					P_loss = pow(10, P_loss / 10);

					tn_matrix += H_ue*(enb->GetLtePhy()->precode_matrix[k])*(enb->GetLtePhy()->allNss_power[k])*(enb->GetLtePhy()->allNss_power[k]).adjoint()*(enb->GetLtePhy()->precode_matrix[k]).adjoint()*H_ue.adjoint()*P_loss;
					//�������������� (Hki��Wi*Pi)*(Hki��Wi*Pi).adjoint �ۼ�	
				}
			}
		}
		else
		{
			for (int i = 0; i < min(Num_Henb_Select,nb_totalHenb - 1); i++)
			{
				HeNodeB* henb = m_henbs->at(ue->ID_Henb[i+1]);
				if (henb->GetLtePhy()->m_num_tx_streams[k]>0)  //�ҵ���ӦƵ��
				{
					Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Hecoefficients[henb->GetIDNetworkNode()][k];           //��С��վ���û����ŵ�����

					double P_loss = ue->GetLteChannel()->HeNodeB_power_loss[henb->GetIDNetworkNode()];
					P_loss = pow(10, P_loss / 10);

					tn_matrix += H_ue*(henb->GetLtePhy()->precode_matrix[k])*(henb->GetLtePhy()->allNss_power[k])*(henb->GetLtePhy()->allNss_power[k]).adjoint()*(henb->GetLtePhy()->precode_matrix[k]).adjoint()*H_ue.adjoint()*P_loss;
					//�������������� (Hki��Wi*Pi)*(Hki��Wi*Pi).adjoint �ۼ�		
				}
			}
		}
		Enb_interference_matrix[k] = tn_matrix;
	}
}