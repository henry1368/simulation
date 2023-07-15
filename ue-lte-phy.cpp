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
UeLtePhy::StartRx(PacketBurst* p)        //txSignal为接收所连接（小）基站的信号
{

	UserEquipment* ue = (UserEquipment*)GetDevice();

	if (ue->GetTargetEnbNode() != NULL )    //用户连接的是基站
	{
		cout << "用户" << GetDevice()->GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << " 开始接收来自宏基站" << ue->GetTargetEnbNode()->GetIDNetworkNode()<<"的LTE数据包"<< std::endl;

		double* m_Sinr = SINR_measure();               //MMSE算出等效SINR
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			if (p->GetPackets().at(0)->m_retransmit_number>0)            //重传次数不为0,则该包的SINR计算需要结合上次的SINR记录              
				m_measuredSinr[i] += m_Sinr[i];       //如果是重传则与上次的SINR合并得出新的SINR
			else
				m_measuredSinr[i] = m_Sinr[i];        //不是重传则记录本次的计算结果
		}

		delete  m_Sinr;

		
		//映射求出最终误包率
		int m_mcsIndexForRx = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs(); 
		double m_eesmSINR = EffectiveSINR::LTE_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);   //输入的SINR为十进制，返回的dB形式
		double m_bler = BLERCurves::LTE_getBLER(m_eesmSINR, m_mcsIndexForRx);             

		double   random_value = ((double)rand()) / RAND_MAX;           //产生随机数,判断此次传输是否成功 


		for (uint32_t i = 0; i < p->GetNPackets(); i++)
		{

			Packet * m_packet = p->GetPackets().at(i);      //取出当前所收的包
			int n = m_packet->m_num_application;            //n为对应用户的业务索引
			int m;                                         //m为对应该业务的缓存队列
			int t = 0;
			std::vector<MacQueue*>* m_macqueue = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
			vector<MacQueue*>::iterator it1;
			for (it1 = m_macqueue->begin(); it1 != m_macqueue->end(); it1++)
			{
				if ((*it1)->GetAppNum() == n)
				{
					m = t;
				}
				t++;                                        //m为enb对应的缓存队列索引
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
//						m_packet->m_retransmit_number++;
//						m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
					}
				}
				else
				{
					cout << "收包失败" << endl;
//					m_packet->m_retransmit_number++;
					/*
					if(m_packet->m_retransmit_number>=重传上限)
					{
					丢弃该包：去掉该次的业务需求
					}
					*/
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
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
						long double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //数据分为2000个喷泉码小包
						int Numofpak = int(PTR[index] / pak_length);                   //收到喷泉码包的个数 =总量/喷泉码包长

						cout << "收到的数据量为:" << m_packet->m_size << ",总数据量为" << PTR[index] << endl;// "bit,喷泉码包个数:" << Numofpak << endl;
						//这一步根据曲线读出接包概率
						double pob = Fountain_decode[Numofpak];
						double Rm = ((double)rand()) / RAND_MAX;
						if (Rm <= pob)
						{
							//解包成功
//							cout << "解包成功" << endl;
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

				ue->LTE_Receive_Data[index] = ue->LTE_Receive_Data[index] + m_packet->m_size;  //数据增加
			}

		}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);           //反馈
	}


	else                                                            //用户连接的是小基站  (ue->GetTargetHenbNode() != NULL)
	{

		cout << "用户" << GetDevice()->GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << " 开始接收来自小基站" << ue->GetTargetHenbNode()->GetIDNetworkNode() << "的LTE数据包" << std::endl;

		double* m_Sinr = SINR_measure();                            //MMSE算出等效SINR
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			if (p->GetPackets().at(0)->m_retransmit_number>0)       //重传次数不为0,则该包的SINR计算需要结合上次的SINR记录              
				m_measuredSinr[i] += m_Sinr[i];                     //如果是重传则与上次的SINR合并得出新的SINR
			else
				m_measuredSinr[i] = m_Sinr[i];                     //不是重传则记录本次的计算结果
		}

		delete   m_Sinr;


		//映射求出最终误包率
		int m_mcsIndexForRx = ue->GetTargetHenbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetMcs();
		double m_eesmSINR = EffectiveSINR::LTE_eesmSINReff(m_measuredSinr, m_mcsIndexForRx, Nss);
		double m_bler = BLERCurves::LTE_getBLER(m_eesmSINR, m_mcsIndexForRx);



		double   random_value = ((double)rand()) / RAND_MAX;       //产生随机数,判断此次传输是否成功 


		for (uint32_t i = 0; i < p->GetNPackets(); i++)
		{

			Packet * m_packet = p->GetPackets().at(i);            //取出当前所收的包
			int n = m_packet->m_num_application;                  //n为对应用户的业务索引
			int m;                                                //m为对应该业务的缓存队列
			int t = 0;
			std::vector<MacQueue*>* m_macqueue = ue->GetTargetHenbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode())->GetQueue();
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
//						m_packet->m_retransmit_number++;
//						m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
					}
				}
				else
				{
					cout << "收包失败" << endl;
//					m_packet->m_retransmit_number++;
					/*
					if(m_packet->m_retransmit_number>=重传上限)
					{
					丢弃该包：去掉该次的业务需求
					}
					*/
//					m_macqueue->at(m)->GetPacketQueue()->front()->m_Retransmit_packet = m_packet;   //设置重传包
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
						double pak_length = (m_macqueue->at(m)->GetPacketQueue()->front()->m_flowNode->m_data_size) / 2000;  //数据分为2000个喷泉码小包
						int Numofpak = int(PTR[index] / pak_length);                   //收到喷泉码包的个数 =总量/喷泉码包长

						cout << "收到的数据量为:" << m_packet->m_size << ",总数据量为" << PTR[index] << endl;// "bit,喷泉码包个数:" << Numofpak << endl;


						//这一步根据曲线读出接包概率
						double pob = Fountain_decode[Numofpak];
						double Rm = ((double)rand()) / RAND_MAX;
						if (Rm <= pob)
						{
							//解包成功
//							cout << "解包成功" << endl;
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

				ue->LTE_Receive_Data[index-1] = ue->LTE_Receive_Data[index-1] + m_packet->m_size;  //数据增加
			}
		}

		CreateAndSendFeedbacks(m_measuredSinr, m_eesmSINR);
	}	
}


double *
UeLtePhy::SINR_measure()
{
	UserEquipment*ue = (UserEquipment*)GetDevice(); //所在用户

	double* eff_Sinr = new double[RBs_FOR_LTE];
	double** m_Sinr = new double*[Nss];
	for (int i = 0; i < Nss; i++)
	{
		m_Sinr[i] = new double[RBs_FOR_LTE];
	}

	Matrix<complex<double>, Nr, Nr>  H_eb_interference[RBs_FOR_LTE];

	GetInterference()->LTE_ComputeInterference(ue, H_eb_interference);    //计算其它基站在该每个频点上的干扰矩阵（Nr×Nr）

	double sigma2 = pow(10, (-174 + 7 + 10 * log10(15 * 12*1000) ) / 10)/1000;            //噪声功率

	for (int i = 0; i < RBs_FOR_LTE; i++)
	{
		if (num_data_streams[i] == 0)
		{
			for (int k = 0; k < Nss; k++)
				m_Sinr[k][i] = 0;				      //每个流上的SINR
		}
		else
		{

			if (ue->GetTargetEnbNode() != NULL)   //连接宏基站
			{
				ENodeB* enb = ue->GetTargetEnbNode();

				double P_loss = ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]; 
				P_loss = pow(10, P_loss / 10);


				Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Ecoefficients[enb->GetIDNetworkNode()][i];  //enb到该用户的信道矩阵Hk（Nr×Nt）
				MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (接收天线数×该用户流数)
				H_hat = H_ue*precode_matrix[i] *sqrt(P_loss)* Nss_power[i];


				//cout << H_ue << endl;
				//cout << precode_matrix[i] << endl;
				//cout << Nss_power[i] << endl;
				//cout<<(ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]);
				//cout << Nss_power[i] / sqrt(ue->GetLteChannel()->eNodeB_power_loss[enb->GetIDNetworkNode()]);

				MatrixXcd H_all = MatrixXcd::Identity(Nr, enb->GetLtePhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (接收天线数×该用户流数)
				H_all = H_ue*enb->GetLtePhy()->precode_matrix[i] * sqrt(P_loss) * enb->GetLtePhy()->allNss_power[i];  //Hk*W*P

				Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //多用户干扰(Nr×Nr)     （单用户时，此矩阵应为零矩阵）

				Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //噪声方阵

				MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE算子

				L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_eb_interference[i]).inverse()); //(用户流数×发射天线数)
				//cout << H_ue << endl;
				//cout << H_hat << endl;
				//cout << H_eb_interference[i] << endl;
				//cout << (H_hat*H_hat.adjoint() + H_eb_interference[i]).inverse()<<endl;


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

				VectorXcd P_Enb = VectorXcd::Zero(num_data_streams[i], 1);
//				P_Enb = ((L_mmse*H_eb_interference[i])*((L_mmse*H_eb_interference[i]).adjoint())).diagonal(); //其它enb干扰,此处为错

				P_Enb = (L_mmse*H_eb_interference[i]*(L_mmse.adjoint())).diagonal(); //此处为对
				VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
				Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_Enb).real().array();    //这里对角线上的元素是实值，由于矩阵属性为复，故求出取实部

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
			else
			{
				HeNodeB* henb = ue->GetTargetHenbNode();
				double P_loss = ue->GetLteChannel()->eNodeB_power_loss[henb->GetIDNetworkNode()];
				P_loss = pow(10, P_loss / 10);

				Matrix<complex<double>, Nr, Nt> H_ue = ue->GetLteChannel()->channel_Hecoefficients[henb->GetIDNetworkNode()][i];  //到该用户的信道矩阵Hk（Nr×Nt）

				MatrixXcd H_hat = MatrixXcd::Identity(Nr, num_data_streams[i]);  //Hk*Wk*Pk (接收天线数×该用户流数)
				H_hat = H_ue*precode_matrix[i] * sqrt(P_loss)*Nss_power[i];

				MatrixXcd H_all = MatrixXcd::Identity(Nr, henb->GetLtePhy()->m_num_tx_streams[i]);   //Hk*Wk*Pk (接收天线数×该用户流数)
				H_all = H_ue*henb->GetLtePhy()->precode_matrix[i] * sqrt(P_loss) * henb->GetLtePhy()->allNss_power[i];  //Hk*W*P

				Matrix<complex<double>, Nr, Nr> H_mu_in = H_all*H_all.adjoint() - H_hat*H_hat.adjoint(); //多用户干扰(Nr×Nr)     （单用户时，此矩阵应为零矩阵）

				Matrix<complex<double>, Nr, Nr> H_noise = sigma2*(MatrixXcd::Identity(Nr, Nr)); //噪声方阵

				MatrixXcd L_mmse = MatrixXcd::Zero(num_data_streams[i], Nr);       //MMSE算子

				L_mmse = H_hat.adjoint()*((H_hat*H_hat.adjoint() + H_noise + H_mu_in + H_eb_interference[i]).inverse()); //(用户流数×发射天线数)

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
			//	P_ap = ((L_mmse*H_eb_interference[i])*((L_mmse*H_eb_interference[i]).adjoint())).diagonal(); //其它AP干扰          //此处有错   

				P_ap = (L_mmse*H_eb_interference[i]*(L_mmse.adjoint())).diagonal(); //其它AP干扰          //此处为对  
				VectorXd Sinr = VectorXd::Zero(num_data_streams[i], 1);
				Sinr = P_ue.real().array() / (P_noise + P_inter_stream + P_inter_ue + P_ap).real().array();    //这里对角线上的元素是实值，由于矩阵属性为复，故求出取实部

				int M = 0;
				for (int N = 0; N < Nss; N++)         //赋值    ,Sinr可能为一个用户多个流
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
		//计算出每个流上的SINR后，在这里进行等效
		for (int k = 0; k < RBs_FOR_LTE; k++)
		{
			eff_Sinr[k] = m_Sinr[0][k];                //注：现在用户均为单流，故无等效
		}

		//删除内存，返回
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
	if (ue->GetTargetEnbNode() != NULL)   //用户连接的是基站
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
	if (ue->GetTargetEnbNode() != NULL)   //用户连接的是基站
	{
		ENodeB::UserEquipmentRecord* Ptr = ue->GetTargetEnbNode()->GetUserEquipmentRecord(ue->GetIDNetworkNode());

		vector<vector<int>> X = Ptr->GetCQI();
		X.at(0).clear();
		X.erase(X.begin());	
		X.push_back(cqi);
		Ptr->SetCQI(X); //缓存cqi


		vector<vector<int>> Y = Ptr->GetPMI();
		Y.at(0).clear();
		Y.erase(Y.begin());
		Y.push_back(cqi);
		Ptr->SetPMI(Y); //缓存pmi


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