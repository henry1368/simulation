#include "NetworkManager.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "ue-lte-phy.h"
#include "lte-mac.h"
#include "enb-lte-phy.h"
#include "henb-lte-phy.h"
#include "lte-bandwidth-manager.h"
#include "packet-burst.h"
#include "interference.h"
#include "Packet.h"
#include "LTE-A_channel.h"
#include "parameter.h"
#include "MacQueue.h"
#include "EffectiveSINR.h"
#include "BLERCurves.h"
#include "Simulator.h"
#include "application.h"
#include "Fountain.h"
#include "CodeBook.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


UeLtePhy::UeLtePhy() 
{
	SetDevice(NULL);
	SetlteBandwidthManager(NULL);
}

UeLtePhy::~UeLtePhy()
{
	Destroy();
}

void
UeLtePhy::TxPowerAllocation(double p)
{
}


void
UeLtePhy::StartRx(PacketBurst* p)        //txSignalΪ���������ӣ�С����վ���ź�
{

	UserEquipment* ue = (UserEquipment*)GetDevice();

	if (ue->GetTargetEnbNode() != NULL )    //�û����ӵ��ǻ�վ
	{
		cout << "�û�" << GetDevice()->GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << " ��ʼ�������Ժ��վ" << ue->GetTargetEnbNode()->GetIDNetworkNode()<<"��LTE���ݰ�"<< std::endl;

		double* m_Sinr = SINR_measure();               //MMSE�����ЧSINR
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			if (p->GetPackets().at(0)->m_retransmit_number>0)            //�ش�������Ϊ0,��ð���SINR������Ҫ����ϴε�SINR��¼              
				m_measuredSinr[i] += m_Sinr[i];       //������ش������ϴε�SINR�ϲ��ó��µ�SINR
			else
				m_measuredSinr[i] = m_Sinr[i];        //�����ش����¼���εļ�����
		}

		delete  m_Sinr;

		
		//ӳ��������������
		int m_mcsIndexForRx = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs(); 
		double m_eesmSINR = EffectiveSINR::LTE_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);   //�����SINRΪʮ���ƣ����ص�dB��ʽ
		double m_bler = BLERCurves::LTE_getBLER(m_eesmSINR, m_mcsIndexForRx);             

		double   random_value = ((double)rand()) / RAND_MAX;           //���������,�жϴ˴δ����Ƿ�ɹ� 


		for (uint32_t i = 0; i < p->GetNPackets(); i++)
		{

			Packet * m_packet = p->GetPackets().at(i);      //ȡ����ǰ���յİ�
			int n = m_packet->m_num_application;            //nΪ��Ӧ�û���ҵ������
			int m;                                         //mΪ��Ӧ��ҵ��Ļ������
			int t = 0;
			std::vector<MacQueue*>* m_macqueue = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
			vector<MacQueue*>::iterator it1;
			for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
			{
				if ((*it1)->GetAppNum() == n)
				{
					m = t;
				}
				t++;                                        //mΪenb��Ӧ�Ļ����������
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
//						m_packet->m_retransmit_number++;
//						m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
					}
				}
				else
				{
					cout << "�հ�ʧ��" << endl;
//					m_packet->m_retransmit_number++;
					/*
					if(m_packet->m_retransmit_number>=�ش�����)
					{
					�����ð���ȥ���ôε�ҵ������
					}
					*/
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
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
						long double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //���ݷ�Ϊ2000����Ȫ��С��
						int Numofpak = int(PTR[index] / pak_length);                   //�յ���Ȫ����ĸ��� =����/��Ȫ�����

						cout << "�յ���������Ϊ:" << m_packet->m_size << ",��������Ϊ" << PTR[index] << endl;// "bit,��Ȫ�������:" << Numofpak << endl;
						//��һ���������߶����Ӱ�����
						double pob = Fountain_decode[Numofpak];
						double Rm = ((double)rand()) / RAND_MAX;
						if (Rm <= pob)
						{
							//����ɹ�
//							cout << "����ɹ�" << endl;
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

				ue->LTE_Receive_Data[index] = ue->LTE_Receive_Data[index] + m_packet->m_size;  //��������
			}

		}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);           //����
	}


	else                                                            //�û����ӵ���С��վ  (ue->GetTargetHenbNode() != NULL)
	{

		cout << "�û�" << GetDevice()->GetIDNetworkNode() << "����ʱ�� t=" << Simulator::Init()->Now() << " ��ʼ��������С��վ" << ue->GetTargetHenbNode()->GetIDNetworkNode() << "��LTE���ݰ�" << std::endl;

		double* m_Sinr = SINR_measure();                            //MMSE�����ЧSINR
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			if (p->GetPackets().at(0)->m_retransmit_number>0)       //�ش�������Ϊ0,��ð���SINR������Ҫ����ϴε�SINR��¼              
				m_measuredSinr[i] += m_Sinr[i];                     //������ش������ϴε�SINR�ϲ��ó��µ�SINR
			else
				m_measuredSinr[i] = m_Sinr[i];                     //�����ش����¼���εļ�����
		}

		delete   m_Sinr;


		//ӳ��������������
		int m_mcsIndexForRx = ue->GetTargetHenbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs();
		double m_eesmSINR = EffectiveSINR::LTE_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);
		double m_bler = BLERCurves::LTE_getBLER(m_eesmSINR, m_mcsIndexForRx);



		double   random_value = ((double)rand()) / RAND_MAX;       //���������,�жϴ˴δ����Ƿ�ɹ� 


		for (uint32_t i = 0; i < p->GetNPackets(); i++)
		{

			Packet * m_packet = p->GetPackets().at(i);            //ȡ����ǰ���յİ�
			int n = m_packet->m_num_application;                  //nΪ��Ӧ�û���ҵ������
			int m;                                                //mΪ��Ӧ��ҵ��Ļ������
			int t = 0;
			std::vector<MacQueue*>* m_macqueue = ue->GetTargetHenbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
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
//						m_packet->m_retransmit_number++;
//						m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
					}
				}
				else
				{
					cout << "�հ�ʧ��" << endl;
//					m_packet->m_retransmit_number++;
					/*
					if(m_packet->m_retransmit_number>=�ش�����)
					{
					�����ð���ȥ���ôε�ҵ������
					}
					*/
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //�����ش���
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
						double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //���ݷ�Ϊ2000����Ȫ��С��
						int Numofpak = int(PTR[index] / pak_length);                   //�յ���Ȫ����ĸ��� =����/��Ȫ�����

						cout << "�յ���������Ϊ:" << m_packet->m_size << ",��������Ϊ" << PTR[index] << endl;// "bit,��Ȫ�������:" << Numofpak << endl;


						//��һ���������߶����Ӱ�����
						double pob = Fountain_decode[Numofpak];
						double Rm = ((double)rand()) / RAND_MAX;
						if (Rm <= pob)
						{
							//����ɹ�
//							cout << "����ɹ�" << endl;
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

				ue->LTE_Receive_Data[index-1] = ue->LTE_Receive_Data[index-1] + m_packet->m_size;  //��������
			}
		}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);
	}	
}


double *
UeLtePhy::SINR_measure()
{
	UserEquipment*ue = (UserEquipment*)GetDevice(); //�����û�

	double* eff_Sinr = new double[RBs_FOR_LTE];
	double** m_Sinr = new double*[Nss];
	for (int i = 0; i < Nss; i++)
	{
		m_Sinr[i] = new double[RBs_FOR_LTE];
	}

	Matrix<complex<double>, Nr, Nr>  H_eb_interference[RBs_FOR_LTE];

	GetInterference()->LTE_ComputeInterference(ue, H_eb_interference);    //����������վ�ڸ�ÿ��Ƶ���ϵĸ��ž���Nr��Nr��

	double sigma2 = pow(10, (-174 + 7 + 10 * log10(15 * 12*1000) ) / 10)/1000;            //��������

	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		if (num_data_streams[i] == 0)
		{
			for (int k = 0; k < Nss; k++)
				m_Sinr[k][i] = 0;				      //ÿ�����ϵ�SINR
		}
		else
		{

			if (ue->GetTargetEnbNode() != NULL)   //���Ӻ��վ
			{
				ENodeB* enb = ue->GetTargetEnbNode();

				double P_loss = ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]; 
				P_loss = pow(10, P_loss / 10);


				Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Ecoefficients[enb->GetIDNetworkNode()][i];  //enb�����û����ŵ�����Hk��Nr��Nt��
				MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (���������������û�����)
				H_hat = H_ue*precode_matrix[i] *sqrt(P_loss)* Nss_power[i];


				//cout << H_ue << endl;
				//cout << precode_matrix[i] << endl;
				//cout << Nss_power[i] << endl;
				//cout<<(ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]);
				//cout << Nss_power[i] / sqrt(ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]);

				MatrixXcd H_all = MatrixXcd::Identity(Nr, enb->GetLtePhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (���������������û�����)
				H_all = H_ue*enb->GetLtePhy()->precode_matrix[i] * sqrt(P_loss) * enb->GetLtePhy()->allNss_power[i];  //Hk*W*P

				Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //���û�����(Nr��Nr)     �����û�ʱ���˾���ӦΪ�����

				Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //��������

				MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE����

				L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_eb_interference[i]).inverse()); //(�û�����������������)
				//cout << H_ue << endl;
				//cout << H_hat << endl;
				//cout << H_eb_interference[i] << endl;
				//cout << (H_hat*H_hat.adjoint() + H_eb_interference[i]).inverse()<<endl;


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

				VectorXcd P_Enb = VectorXcd::Zero(num_data_streams[i], 1);
//				P_Enb = ((L_mmse*H_eb_interference[i])*((L_mmse*H_eb_interference[i]).adjoint())).diagonal(); //����enb����,�˴�Ϊ��

				P_Enb = (L_mmse*H_eb_interference[i]*(L_mmse.adjoint())).diagonal(); //�˴�Ϊ��
				VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
				Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_Enb).real().array();    //����Խ����ϵ�Ԫ����ʵֵ�����ھ�������Ϊ���������ȡʵ��

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
			else
			{
				HeNodeB* henb = ue->GetTargetHenbNode();
				double P_loss = ue->GetLteChannel()->eNodeB_power_loss[henb->GetIDNetworkNode()];
				P_loss = pow(10, P_loss / 10);

				Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Hecoefficients[henb->GetIDNetworkNode()][i];  //�����û����ŵ�����Hk��Nr��Nt��

				MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (���������������û�����)
				H_hat = H_ue*precode_matrix[i] * sqrt(P_loss)*Nss_power[i];

				MatrixXcd H_all = MatrixXcd::Identity(Nr, henb->GetLtePhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (���������������û�����)
				H_all = H_ue*henb->GetLtePhy()->precode_matrix[i] * sqrt(P_loss) * henb->GetLtePhy()->allNss_power[i];  //Hk*W*P

				Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //���û�����(Nr��Nr)     �����û�ʱ���˾���ӦΪ�����

				Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //��������

				MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE����

				L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_eb_interference[i]).inverse()); //(�û�����������������)

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
			//	P_ap = ((L_mmse*H_eb_interference[i])*((L_mmse*H_eb_interference[i]).adjoint())).diagonal(); //����AP����          //�˴��д�   

				P_ap = (L_mmse*H_eb_interference[i]*(L_mmse.adjoint())).diagonal(); //����AP����          //�˴�Ϊ��  
				VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
				Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_ap).real().array();    //����Խ����ϵ�Ԫ����ʵֵ�����ھ�������Ϊ���������ȡʵ��

				int M = 0;
				for (int N = 0; N < Nss; N++)         //��ֵ    ,Sinr����Ϊһ���û������
				{
					if (m_channelsForRx[N][i]>0)
					{
						m_Sinr[N][i] = Sinr(M);
						M++;
					}
				}


			}
		}
	}
		//�����ÿ�����ϵ�SINR����������е�Ч
		for (int k = 0; k < RBs_FOR_LTE; k++)
		{
			eff_Sinr[k] = m_Sinr[0][k];                //ע�������û���Ϊ���������޵�Ч
		}

		//ɾ���ڴ棬����
		for (int i = 0; i < Nss; i++)
			delete[] m_Sinr[i];
		delete[]  m_Sinr;
		return eff_Sinr;

}


void
UeLtePhy::CreateAndSendFeedbacks(double sinr[], double m_eesmSINR)
{

	vector<int> m_cqi = CreateCqiFeedbacks(m_eesmSINR);
	vector<int> m_pmi = CreatePMIFeedbacks(sinr);
	SendFeedbacks(m_cqi, m_pmi);
}

vector<int>
UeLtePhy::CreateCqiFeedbacks(double m_eesmSINR)
{
	UserEquipment*ue = (UserEquipment*)GetDevice();
	vector<int> cqi;
	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		cqi.push_back(ue->GetLteMac()->GetAmcModule()->GetCQIFromSinr(m_eesmSINR));
	}

	return cqi;
}

vector<int>
UeLtePhy::CreatePMIFeedbacks(double sinr[])
{
	UserEquipment*ue = (UserEquipment*)GetDevice();
	vector<int> pmi;
	if (ue->GetTargetEnbNode() != NULL)   //�û����ӵ��ǻ�վ
	{
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Ecoefficients[ue->GetTargetEnbNode()->GetIDNetworkNode()][i];
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
						detcapacity = (I + X*sinr[i]/2).determinant();
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
			pmi.push_back(pmi_) ;
		}
	}
	else
	{
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Hecoefficients[ue->GetTargetHenbNode()->GetIDNetworkNode()][i];
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
	}
	return pmi;
}

void
UeLtePhy::SendFeedbacks(vector<int> cqi, vector<int> pmi)
{
	UserEquipment*ue = (UserEquipment*)GetDevice();
	if (ue->GetTargetEnbNode() != NULL)   //�û����ӵ��ǻ�վ
	{
		ENodeB::UserEquipmentRecord* Ptr = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode());

		vector<vector<int>> X = Ptr->GetCQI();
		X.at(0).clear();
		X.erase(X.begin());	
		X.push_back(cqi);
		Ptr->SetCQI(X); //����cqi


		vector<vector<int>> Y = Ptr->GetPMI();
		Y.at(0).clear();
		Y.erase(Y.begin());
		Y.push_back(cqi);
		Ptr->SetPMI(Y); //����pmi


	}
	else
	{
		HeNodeB::UserEquipmentRecord* Ptr = ue->GetTargetHenbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode());

		vector<vector<int>> X = Ptr->GetCQI();
		X.at(0).clear();
		X.erase(X.begin());
		X.push_back(cqi);
		Ptr->SetCQI(X);

		vector<vector<int>> Y = Ptr->GetPMI();
		Y.at(0).clear();
		Y.erase(Y.begin());
		Y.push_back(cqi);
		Ptr->SetPMI(Y);
	}

}