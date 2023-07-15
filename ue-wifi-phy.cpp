
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
UeWifiPhy::StartRx(PacketBurst* p)           //һ���û��������ݰ��������ǿ�ҵ����ߵ�ҵ��粻ͬ���ݿ飬Ŀǰ�ǵ�ҵ�����ݿ飩
{

	UserEquipment* ue = (UserEquipment*)GetDevice();

	DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_RxStart = Simulator::Init()->Now();  //��¼��ʼ����ʱ��
	
	cout << "�û�" << GetDevice()->GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << " ��ʼ��������AP" << ue->GetTargetAPNode()->GetIDNetworkNode()<<"��WIFI���ݰ�:"<<std::endl;

	double* m_Sinr = SINR_measure();               //MMSE�����ЧSINR
	for (int i = 0; i < RBs_FOR_WIFI; i++)
	{
		if (p->GetPackets().at(0)->m_retransmit_number>0)            //�ش�������Ϊ0,��ð���SINR������Ҫ����ϴε�SINR��¼              
			m_measuredSinr[i] += m_Sinr[i];                          //������ش������ϴε�SINR�ϲ��ó��µ�SINR
		else
			m_measuredSinr[i] = m_Sinr[i];                           //�����ش����¼���εļ�����
	}

	delete  m_Sinr;



	int m_mcsIndexForRx = ue->GetTargetAPNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs();
	double m_eesmSINR = EffectiveSINR::WIFI_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);
	double m_bler = BLERCurves::WIFI_getBLER(m_eesmSINR, m_mcsIndexForRx, Nss);             //������������

//	m_bler = 0.001;
	double   random_value = ((double)rand()) / 0x7fff;           //���������,�жϴ˴δ����Ƿ�ɹ� 


	for (uint32_t i=0; i < p->GetNPackets();i++)
	{

		Packet * m_packet = p->GetPackets().at(i);      //ȡ����ǰ���յİ�
		int n = m_packet->m_num_application;            //nΪ��Ӧ�û���ҵ������
		int m;                                         //mΪ��Ӧ��ҵ��Ļ������
		int t = 0;
		std::vector<MacQueue*>* m_macqueue = ue->GetTargetAPNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
		vector<MacQueue*>::iterator it1;
		for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
		{
			if ((*it1)->GetAppNum() == n)
			{
				m = t;
			}
			t++;                                        //mΪAP��Ӧ�Ļ����������
		}


		if (random_value <= m_bler)                     //ʧ��
		{

			//��Ϊ��Ȫ��İ��ͷ���Ȫ��İ����д���

			if (m_packet->m_is_FountainCode)
			{
				// step1�� �жϸð����ڵ�ҵ��������ͨ����һ��������
				int index = m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_index; //ȡ����Ӧ������
				int *ptr = ue->GetApplist()->at(n)->m_flag_ptr;
				if (ptr[index] == 1)                                         //��Ӧ�ı�־λ,��ʾ�������ѱ�����
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();        //�������,��֤������������
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

				}
				else
				{
					cout << "�հ�ʧ��" << endl;
//					m_packet->m_retransmit_number++;
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
				}
			}
			else
			{
				cout << "�հ�ʧ��" << endl;
//				m_packet->m_retransmit_number++;
				/*
				if(m_packet->m_retransmit_number>=�ش�����)
				{
				�����ð���ȥ���ôε�ҵ������
				}
				*/
//				m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
			}
		}

		else                                     //�ɹ�       
		{
			cout << "�հ��ɹ�,";
			if (m_packet->m_is_FountainCode)                             //��Ȫ���
			{
				// step1�� �жϸð����ڵ�ҵ��������ͨ����һ��������
				int index = m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_index; //ȡ����Ӧ������
				int *ptr = ue->GetApplist()->at(n)->m_flag_ptr;
				long double *PTR = ue->GetApplist()->at(n)->m_Fountain_data;
				if (ptr[index] == 1)                                         //��Ӧ�ı�־λ,��ʾ�������ѱ�����
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();        //�������,��֤������������
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

				}
				else
				{
					PTR[index] = PTR[index] + m_packet->m_size;     //�յ����ݵ�����
					double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //���ݷ�Ϊ2000����Ȫ��С���������Ȫ�����
					int Numofpak =int( PTR[index] / pak_length);                   //�յ���Ȫ����ĸ��� =����/��Ȫ�����

					cout << "�յ���������Ϊ:" << m_packet->m_size << ",��������Ϊ" << PTR[index] << endl;// "bit,��Ȫ�������:" << Numofpak << endl;
					//��һ���������߶����Ӱ�����
					double pob = Fountain_decode[Numofpak];
					double Rm = ((double)rand()) / 0x7fff;
					if (Rm <= pob)
					{
						//����ɹ�
	//					cout << "����ɹ�" << endl;
						ptr[index] = 1;   //�����û��Ľ�����Ϣ
						m_macqueue->at(m)->GetPacketQueue()->pop_front();          //�������,��֤������������
						if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
							m_macqueue->erase(m_macqueue->begin() + m);

						ue->GetApplist()->at(n)->GetDatalist()->pop_front();        //����û����������󣬱�֤������������
						if (ue->GetApplist()->at(n)->GetDatalist()->size() == 0)
							ue->GetApplist()->erase(ue->GetApplist()->begin() + n);
					}
				}
			}

			else                  //����Ȫ��
			{
				if (m_packet->m_isAFragment == false || m_packet->m_isTheLatestFragment == true)    //�ð�����˴����ݵ��������
				{
					m_macqueue->at(m)->GetPacketQueue()->pop_front();          //�������,��֤������������
					if (m_macqueue->at(m)->GetPacketQueue()->size() == 0)
						m_macqueue->erase(m_macqueue->begin() + m);

					ue->GetApplist()->at(n)->GetDatalist()->pop_front();        //����û����������󣬱�֤������������
					if (ue->GetApplist()->at(n)->GetDatalist()->size() == 0)
						ue->GetApplist()->erase(ue->GetApplist()->begin() + n);

				}
				else
				{
					cout << "�˰�Ϊ��ͨ��Ƭ�����������һ����" << endl;
					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = NULL;
				}
			}
       
          //�ڴ����洢��������
			int index = int(Simulator::Init()->Now() / 1000);                     //�������ڼ���TTI

			ue->WIFI_Receive_Data[index] = ue->WIFI_Receive_Data[index]+ m_packet->m_size;  //��������
		}
	}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);           //���㷴��

		DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_RxEnd = Simulator::Init()->Now() + 44;   //��¼���ڴؽ��ս���ʱ��

		DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_lastNavDuration = DcfManager::Init()->GetCluster(ue->GetTargetAPNode())->m_lastNavStart+224;  //��¼���ڴ�NAVʱ��

		ue->GetTargetAPNode()->GetApMac()->GetDcfstate()->ResetState();   //��������AP�˱�״̬

}




double *
UeWifiPhy::SINR_measure()
{
	UserEquipment*ue = (UserEquipment*)GetDevice(); //�����û�
	AP* m_ap = ue->GetTargetAPNode();               //����ap

	double* eff_Sinr = new double[RBs_FOR_WIFI];
	double** m_Sinr = new double*[Nss];
	for (int i = 0; i < Nss; i++)
	{
		m_Sinr[i] = new double[RBs_FOR_WIFI];         
	}

	Matrix<complex<double>, Nr, Nr>  H_ap_interference[RBs_FOR_WIFI]; 
	
	GetInterference()->WIFI_ComputeInterference(ue, H_ap_interference);    //��������AP�ڸ�ÿ��Ƶ���ϵĸ��ž���Nr��Nr��

	   
   double sigma2 = pow(10, (-174 + 7 + 10 * log10(312.5 * 1000) ) / 10)/1000;            //��������

   for (int i = 0; i < RBs_FOR_WIFI; i++)
   {
	   if (num_data_streams[i] == 0)
	   {
		   for (int k = 0; k < Nss; k++)
			   m_Sinr[k][i] = 0;				      //ÿ�����ϵ�SINR
	   }
	   else
	   {
		   Matrix<complex<double>, Nr, Nt> H_ue = ue->GetWifiChannel()->channel_APcoefficients[m_ap->GetIDNetworkNode()][i];  //ap�����û����ŵ�����Hk��Nr��Nt��

		   MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (���������������û�����)
		   H_hat = H_ue*precode_matrix[i] * Nss_power[i];

		   MatrixXcd H_all = MatrixXcd::Identity(Nr, m_ap->GetWifiPhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (���������������û�����)
		   H_all = H_ue* m_ap->GetWifiPhy()->precode_matrix[i] * m_ap->GetWifiPhy()->allNss_power[i];  //Hk*W*P

		   Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //���û�����(Nr��Nr)     �����û�ʱ���˾���ӦΪ�����

		   Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //��������

		   MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE����

		   L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_ap_interference[i]).inverse()); //(�û�����������������)

		   //����SINR:

		   MatrixXcd D = MatrixXcd::Zero(num_data_streams[i], num_data_streams[i]);
		   MatrixXcd diag_D = MatrixXcd::Zero(num_data_streams[i], num_data_streams[i]);
		   D = L_mmse*H_hat;
		   diag_D = D.diagonal().asDiagonal();

		   VectorXcd P_ue = VectorXcd::Zero(num_data_streams[i], 1);
		   P_ue = (diag_D*(diag_D.adjoint())).diagonal();          //�����źŹ���

		   VectorXcd P_inter_stream = VectorXcd::Zero(num_data_streams[i], 1);
		   P_inter_stream = (((D - diag_D)*(D - diag_D)).adjoint()).diagonal();  //�������

		   VectorXcd P_inter_ue = VectorXcd::Zero(num_data_streams[i], 1);
	       P_inter_ue = (L_mmse*H_mu_in*((L_mmse*H_mu_in).adjoint())).diagonal(); //���û�����

		   VectorXcd P_noise = VectorXcd::Zero(num_data_streams[i], 1);
		   P_noise = ((L_mmse*(L_mmse.adjoint())).diagonal())*sigma2;               //��������

		   VectorXcd P_ap = VectorXcd::Zero(num_data_streams[i], 1);
//		   P_ap = ((L_mmse*H_ap_interference[i])*((L_mmse*H_ap_interference[i]).adjoint())).diagonal(); //����AP����       //�����д�   

		   P_ap = ((L_mmse*H_ap_interference[i])*(L_mmse.adjoint())).diagonal(); //����AP����       //�����д�
		   VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
		   Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_ap).real().array();    //����Խ����ϵ�Ԫ����ʵֵ�����ھ�������Ϊ���������ȡʵ��

		   int M = 0;
		   for (int N = 0; N < Nss; N++)         //��ֵ
		   {
			   if (m_channelsForRx[N][i]>0)
			   {
				   m_Sinr[N][i] = Sinr(M);         
				   M++;
			   }	  
		  }


	   }
   }

	//�����ÿ�����ϵ�SINR����������е�Ч
	for (int k = 0; k < RBs_FOR_WIFI; k++)
	{
		eff_Sinr[k] = m_Sinr[0][k];                //ע�������û���Ϊ���������޵�Ч
	}

	//ɾ���ڴ棬����
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
UeWifiPhy::CreateAndSendFeedbacks(double sinr[], double m_eesmSINR)           //���㷴��
{
	vector<int> m_cqi = CreateCqiFeedbacks(m_eesmSINR);
	vector<int> m_pmi = CreatePMIFeedbacks(sinr);
	SendFeedbacks(m_cqi, m_pmi);
}

vector<int>
UeWifiPhy::CreateCqiFeedbacks(double m_eesmSINR)
{
	double WIFI_SINRForCQIIndex[10] = { -2.25, 2, 4.5, 8, 11.5, 15.75, 17.5, 18.25, 23.5, 24.25 };//���ӳ���
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

//����ȷ�����CQI
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
	//Ptr->GetCQI().push_back(cqi);                          //����cqi

	vector<vector<int>> X = Ptr->GetCQI();
	X.at(0).clear();
	X.erase(X.begin());
	X.push_back(cqi);
	Ptr->SetCQI(X);



	//Ptr->GetPMI().at(0).clear();
	//Ptr->GetPMI().erase(Ptr->GetPMI().begin());           //����pmi
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
