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
	WIFI中不同的AP采用不同的信道，因此其频段选择存在差异，不同的AP间可能存在着部分的频率交叠，
	因此在计算干扰时，需要找出其他AP存在着频率复用，且由于时间不同步，仅当前时刻处于别的wifi簇 
	的发送数据时间区间内时才存在干扰。
	*/

	//初始化所有频点上的其它AP干扰
	for (int i = 0; i<RBs_FOR_WIFI; i++)
		AP_interference_matrix[i] = MatrixXcd::Zero(Nr, Nr);

	double time_now = Simulator::Init()->Now();                       //当前时刻
	vector<AP*> *m_aps = NetworkManager::Init()->GetAPContainer();    //所有ap
	for (int k = 0; k < RBs_FOR_WIFI; k++)
		{
			double frequence = ue->GetWifiPhy()->GetwifiBandwidthManager()->GetDLSubChannels().at(k);   //频点
			Matrix<complex<double>, Nr, Nr> tn_matrix = AP_interference_matrix[k];                       //初始化为零矩阵

			for (int t = 0; t < nb_apingroup-1; t++)
			{
				AP* ap = m_aps->at(ue->ID_AP[t + 1]);                              //同群内的其它AP，
				DcfState_Cluster *L = DcfManager::Init()->GetCluster(ap);
				if (L->m_RxStart <= time_now && L->m_RxEnd >= time_now)            //发送事件重合且存在频率复用
				{
					for (int m = 0; m < RBs_FOR_WIFI; m++)
					{
						if (ap->GetWifiPhy()->GetwifiBandwidthManager()->GetDLSubChannels().at(m) == frequence && ap->GetWifiPhy()->m_num_tx_streams[m]>0)  //找到对应频点
						{
							double P_loss = ue->GetWifiChannel()->AP_power_loss[ap->GetIDNetworkNode()];
							P_loss = pow(10, P_loss / 10);
							Matrix<complex<double>, Nr, Nt> H_ue = ue->GetWifiChannel()->channel_APcoefficients[ap->GetIDNetworkNode()][m];           //该AP到用户的信道矩阵
							tn_matrix += H_ue*(ap->GetWifiPhy()->precode_matrix[m])*(ap->GetWifiPhy()->allNss_power[m])*(ap->GetWifiPhy()->allNss_power[m]).adjoint()*(ap->GetWifiPhy()->precode_matrix[m]).adjoint()*H_ue.adjoint()*P_loss;
							break;                           //这里计算的是所有 (Hki×Wi*Pi)*(Hki×Wi*Pi).adjoint 累加
						}
					}
				}
			}
			AP_interference_matrix[k] = tn_matrix;
		}
   
}






void
Interference::LTE_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>* Enb_interference_matrix)      //计算其它宏基站、小基站干扰
{
	/*
	所有扇区和小基站均复用全频带
	*/

	for (int i = 0; i< RBs_FOR_LTE; i++)
		Enb_interference_matrix[i] = MatrixXcd::Zero(Nr, Nr);


	vector<ENodeB*> *m_enbs = NetworkManager::Init()->GetENodeBContainer();       //所有宏基站
	vector<HeNodeB*> *m_henbs = NetworkManager::Init()->GetHeNodeBContainer();    //所有小基站

	for (int k = 0; k < RBs_FOR_LTE; k++)
	{

		Matrix<complex<double>, Nr, Nr> tn_matrix = Enb_interference_matrix[k];                        //初始化为零矩阵
		vector<ENodeB*>::iterator it;
		vector<HeNodeB*>::iterator it1;

		if (ue->GetTargetEnbNode() != NULL)
		{
			for (int i = 0; i < min(Num_Enb_Select, nb_totalEnb-1); i++)
			{
				ENodeB* enb = m_enbs->at(ue->ID_Enb[i+1]);
				if (enb->GetLtePhy()->m_num_tx_streams[k]>0)           //存在干扰
				{
					Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Ecoefficients[enb->GetIDNetworkNode()][k];           //该宏基站到用户的信道矩阵
					/*
					cout << H_ue << endl << endl;
					cout << enb->GetLtePhy()->precode_matrix[k] << endl << endl;
					cout << enb->GetLtePhy()->allNss_power[k]<<endl<<endl;                 //有错
					cout << (enb->GetLtePhy()->allNss_power[k]).adjoint() << endl << endl; //有错
					cout << (enb->GetLtePhy()->precode_matrix[k]).adjoint() << endl << endl;
					cout << H_ue.adjoint() << endl << endl;
					*/

					double P_loss = ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]; 
					P_loss = pow(10, P_loss / 10);

					tn_matrix += H_ue*(enb->GetLtePhy()->precode_matrix[k])*(enb->GetLtePhy()->allNss_power[k])*(enb->GetLtePhy()->allNss_power[k]).adjoint()*(enb->GetLtePhy()->precode_matrix[k]).adjoint()*H_ue.adjoint()*P_loss;
					//这里计算的是所有 (Hki×Wi*Pi)*(Hki×Wi*Pi).adjoint 累加	
				}
			}
		}
		else
		{
			for (int i = 0; i < min(Num_Henb_Select,nb_totalHenb - 1); i++)
			{
				HeNodeB* henb = m_henbs->at(ue->ID_Henb[i+1]);
				if (henb->GetLtePhy()->m_num_tx_streams[k]>0)  //找到对应频点
				{
					Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Hecoefficients[henb->GetIDNetworkNode()][k];           //该小基站到用户的信道矩阵

					double P_loss = ue->GetLteChannel()->HeNodeB_power_loss[henb->GetIDNetworkNode()];
					P_loss = pow(10, P_loss / 10);

					tn_matrix += H_ue*(henb->GetLtePhy()->precode_matrix[k])*(henb->GetLtePhy()->allNss_power[k])*(henb->GetLtePhy()->allNss_power[k]).adjoint()*(henb->GetLtePhy()->precode_matrix[k]).adjoint()*H_ue.adjoint()*P_loss;
					//这里计算的是所有 (Hki×Wi*Pi)*(Hki×Wi*Pi).adjoint 累加		
				}
			}
		}
		Enb_interference_matrix[k] = tn_matrix;
	}
}