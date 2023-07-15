#include <cmath>
#include "MadridGrid-propagation-model.h"
#include "lte-propagation-loss-model.h"
#include "NetworkManager.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "Building.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "LTE-A_channel.h"
#include  "Mobility.h"
#include "AntennaArray.h"



/*
�������ִ���ģʽ��O2O��O2I��I2O��I2I�µ��ŵ���߶ȼ���
*/

double RandUni_1();
double RandNorm_1();
double wrapAngleDistribution_1(double theta);


double B1_NLOS_PL(double Dist1, double Dist2, double PL_bp, double CenterFrequency, double H_bs, double H_ms);   //B1�����������зǿ���·�����
void B4_NLOS_Azimuth(UserEquipment* ms, HeNodeB* bs, double * dist_out, double * dist_in, double * cos_theta);       //����B4(UMi)�����⵽���ڵľ��룬�Ƕȵ�
void C4_NLOS_Azimuth(UserEquipment* ms, ENodeB* bs, double * dist_out, double * dist_in);       //����C4(UMa)�����⵽���ڵľ��룬�Ƕȵ�

MadridGrid_PropagationLossModel::MadridGrid_PropagationLossModel()
{
	SetPropagationType(LtePropagationLossModel::TYPE_TC2);
}

MadridGrid_PropagationLossModel::~MadridGrid_PropagationLossModel()
{}

void
MadridGrid_PropagationLossModel::Compute_PowerLoss(UserEquipment* m_src, vector<ENodeB*> *m_dst)
{
	if (m_dst->size() > 0)     //���վ
	{

		/*���վ3������֮��Ĵ�߶ȳ�������������Ϊ��λ�ǶȲ�һ��֮�⣬��������·����Ӱ����͸��ĵȾ�һ��*/
		for (int i = 0; i < nb_cell; i++)
		{
			ENodeB* enb = m_dst->at(3 * i);
			//������
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i] = Compute_isLOS(m_src, enb);
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i + 1] = m_src->GetLteChannel()->eNodeB_isLOS[3 * i];
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i + 2] = m_src->GetLteChannel()->eNodeB_isLOS[3 * i];

			//·��
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i] = Compute_PathLoss(m_src, enb);
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_path_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_path_loss[3 * i];

			//��͸���
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i] = Compute_PenetrationLoss(m_src, enb);
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i];

			//��Ӱ˥��
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i] = Compute_ShadowFading(m_src, enb);
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i];

			//��������
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, enb);
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, m_dst->at(3 * i + 1));
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, m_dst->at(3 * i + 2));       //��ͬ�����������治ͬ

			//LSPS����
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 1][0] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][0];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 1][1] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][1];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 1][2] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][2];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 1][3] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][3];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 1][4] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][4];

			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 2][0] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][0];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 2][1] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][1];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 2][2] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][2];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 2][3] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][3];
			m_src->GetLteChannel()->eNodeB_LSPs[3 * i + 2][4] = m_src->GetLteChannel()->eNodeB_LSPs[3 * i][4];

		}

		for (int i = 0; i < nb_totalEnb; i++)
		{
			m_src->GetLteChannel()->eNodeB_power_loss[i] = m_src->GetLteChannel()->eNodeB_antennaGain[i] - m_src->GetLteChannel()->eNodeB_path_loss[i]
				- m_src->GetLteChannel()->eNodeB_shadow_loss[i] - m_src->GetLteChannel()->eNodeB_penetration_loss[i] - 2;    //2dBΪ�������
		}

	}
}



void
MadridGrid_PropagationLossModel::Compute_PowerLoss(UserEquipment* m_src, vector<HeNodeB*> *m_dst)
{
	if (m_dst->size() > 0)     //���վ
	{

		for (unsigned int i = 0; i < m_dst->size(); i++)
		{
			HeNodeB* henb = m_dst->at(i);
			//������
			m_src->GetLteChannel()->HeNodeB_isLOS[i] = Compute_isLOS(m_src, henb);

			//·��
			m_src->GetLteChannel()->HeNodeB_path_loss[i] = Compute_PathLoss(m_src, henb);

			//��͸���
			m_src->GetLteChannel()->HeNodeB_penetration_loss[i] = Compute_PenetrationLoss(m_src, henb);

			//��Ӱ˥��
			m_src->GetLteChannel()->HeNodeB_shadow_loss[i] = Compute_ShadowFading(m_src, henb);

			//��������
			m_src->GetLteChannel()->HeNodeB_antennaGain[i] = Compute_AntennaGain(m_src, henb);

			//���
			m_src->GetLteChannel()->HeNodeB_power_loss[i] = m_src->GetLteChannel()->HeNodeB_antennaGain[i] - m_src->GetLteChannel()->HeNodeB_path_loss[i]
				- m_src->GetLteChannel()->HeNodeB_shadow_loss[i] - m_src->GetLteChannel()->HeNodeB_penetration_loss[i] - 2;    //2dBΪ�������
		}
	}
}



bool
MadridGrid_PropagationLossModel::Compute_isLOS(UserEquipment* m_src, NetworkNode* m_dst)                //�Ӿ����
{
	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)      //���վ��վλ������¥��,����UMa��
	{
	   if (m_src->GetPostate() == m_Indoor)
		     return true;
	   else                               //outdoor 
	 {
		double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance
		double pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 63.0)) + exp(-d / 63.0);
		return (RandUni_1() < pLOS);
	 }	
	}

	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)   //���վ��վλ�����⣬����UMi��
	{
		if (m_src->GetPostate() == m_Indoor)
			return true;
		else                               //outdoor 
		{
			double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance
			double pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
			return (RandUni_1() < pLOS);
		}
	}


	else        //(m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_UE)   //�û� D2D
	{
		return true;
	}

}


double
MadridGrid_PropagationLossModel::Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst) //����pathloss
{
	bool m_isLOS;
	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
	{
		m_isLOS = m_src->GetLteChannel()->eNodeB_isLOS[m_dst->GetIDNetworkNode()];
		fc = 2.0;
		E_medi.fc_m = fc;                                             //carrier frequency
		const double c = 300000000;
		double BS_HEIGHT = m_dst->Getposition()->GetPositionZ();
		double MS_HEIGHT = m_src->Getposition()->GetPositionZ();
		double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;   //break point distance
		double d = distance(m_src->Getposition(), m_dst->Getposition());     //the distance

		if (m_src->GetPostate() == m_Indoor)             //UMa O2I "C4",  NLOS
		{
			double Dist_out = 0;//outdoor distance
			double Dist_in = 0; //indoor distance
			C4_NLOS_Azimuth(m_src, (ENodeB*)m_dst, &Dist_out, &Dist_in);     //����UMa��������վ�������û������������
			double loss = (44.9 - 6.55*log10(BS_HEIGHT))*log10(Dist_out + Dist_in) + 34.46 + 5.83*log10(BS_HEIGHT) + 23*log10(fc / 5);
			loss += 17.4 + 0.5*Dist_in - 0.8*MS_HEIGHT;
			return loss;
		}

		else                            //UMa O2O "C2"
		{	
			if (m_isLOS)              //LOS         
			{
				if (d >= 10 && d < dBP)
					return  26 * log10(d) + 39 + 20 * log10(fc/5);
				else if (d >= dBP && d < 5000)
					return  40 * log10(d) + 13.47 - 14 * log10(BS_HEIGHT - 1) - 14 * log10(MS_HEIGHT - 1) + 6 * log10(fc / 5);
				else
					return 0;       //assert(false) �������еĿؼ�·��������ֵ
			}
			else                    //NLOS
			{
				if (d >= 10 && d < 5000)
					return  (44.9 - 6.55*log10(BS_HEIGHT))*log10(d) + 34.46 + 5.83*log10(MS_HEIGHT) + 23*log10(fc / 5);
				else
					return 0;       //assert(false) �������еĿؼ�·��������ֵ
			}
		}
	}
	
	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
        m_isLOS = m_src->GetLteChannel()->HeNodeB_isLOS[m_dst->GetIDNetworkNode()];
		fc = 3.5;
		HE_medi.fc_m = fc;                                             //carrier frequency
		const double c = 300000000;
		double BS_HEIGHT = m_dst->Getposition()->GetPositionZ();
		double MS_HEIGHT = m_src->Getposition()->GetPositionZ();
		double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;   //break point distance
		double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance

		if (m_src->GetPostate() == m_Indoor)             //UMi O2I "B4"
		{	
			double Dist_out = 0;//outdoor distance
			double Dist_in = 0; //indoor distance
			double Cos_Theta = 0;   //Cos(angle) from the BS to the normal of the wall
			B4_NLOS_Azimuth(m_src, (HeNodeB*)m_dst, &Dist_out, &Dist_in, &Cos_Theta);      //����UMi������С��վ�������û������������ͽǶȵ�

			if (3 < Dist_out + Dist_in&&Dist_out + Dist_in < 1000)
			{
				//Free space loss
				double FSL = 32.4 + 20 * log10((Dist_out + Dist_in) / 1000) + 20 * log10(fc * 1000);
				//Outdoor loss
				double LossOut = max((41 + 20 * log10(fc / 5) + 22.7*log10(Dist_out + Dist_in)), FSL);
				//indoor loss
				double LossIn = 0.5*Dist_in;    //alpha is 0.5 dB / meter
				//Through wall loss
				double LossWall = 14 + 15 * pow((1 - Cos_Theta), 2);
				//Total loss
				return  LossOut + LossIn + LossWall;
			}
			else
				return 0;                    // assert(false);�������еĿؼ�·��������ֵ
		}

		else                    //UMi O2O "B1"
		{
			if (m_isLOS)          //LOS   
			{
				if (d >= 0 && d < dBP)
					return  22.7 * log10(d) + 41 + 20 * log10(fc / 5);
				else if (d >= dBP && d < 5000)
					return  40 * log10(d) + 9.45 - 17.3 * log10(BS_HEIGHT - 1) - 17.3 * log10(MS_HEIGHT - 1) + 2.7 * log10(fc / 5);
				else
					return 0;       //assert(false) �������еĿؼ�·��������ֵ
			}
			else   
			{			
				double d1 = max(m_dst->Getposition()->GetPositionX() - m_src->Getposition()->GetPositionX(), 
					m_src->Getposition()->GetPositionX() - m_dst->Getposition()->GetPositionX());     //|x1-x2|
				double d2 = max(m_dst->Getposition()->GetPositionY() - m_src->Getposition()->GetPositionY(),
					m_src->Getposition()->GetPositionY() - m_dst->Getposition()->GetPositionY());     //|y1-y2|
				return  min(B1_NLOS_PL(d1, d2, dBP, fc, BS_HEIGHT, MS_HEIGHT), B1_NLOS_PL(d2, d1, dBP, fc, BS_HEIGHT, MS_HEIGHT));
			}
		}

	}

	else             // (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_UE)      //D2D
	{
		return 0;
	}
}

double
MadridGrid_PropagationLossModel::Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst)      //��͸���
{
	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB || m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
		if (m_src->GetPostate() == m_Indoor)
		{
			vector<Building*>* Bd = NetworkManager::Init()->GetBuildingContainer();
			std::vector<Building*>::iterator iter;
			for (iter = Bd->begin(); iter != Bd->end(); iter++)
			{
				Building* b = (*iter);

				if ((b->IsInThisBuilding(m_src)) && (b->IsInThisBuilding(m_dst)))           //�ҵ��û�����¥��
				{
					int Nf = b->GetFloors() - int(m_src->Getposition()->GetPositionZ() / b->GetRoomheight());   //��վ���û�֮�䴩͸��¥����
					return 17 + 4 * (Nf - 1);
				}
			}

	       return   0.5 * 25 * RandUni_1() + 20;
		}

		else if (m_src->GetPostate() == m_Vehicle)
			  return    9 + 5.*RandNorm_1();
		  else
			  return 0;
	}

	else                 //if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_UE)      //D2D
	{
		return 0;//an  additional  loss  of  10  dB  will  be  added  to  the  propagation  loss  whenever devices have others in between.
			
	}

}


double
MadridGrid_PropagationLossModel::Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst)       //��Ӱ˥��
{
	bool m_isLOS;
	double DS_mu, DS_sigma, ASD_mu, ASD_sigma, ASA_mu, ASA_sigma, SF_sigma, K_mu, K_sigma;
	double sqrt_corr_matrix[5][5] = { 0.0 };
	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
	{
		m_isLOS = m_src->GetLteChannel()->eNodeB_isLOS[m_dst->GetIDNetworkNode()];
		if (LOSflag == 1)//���к�С���Ŀ��ӳ���
		{
			sqrt_corr_matrix[0][0] = 0.8063; sqrt_corr_matrix[0][1] = 0.2450; sqrt_corr_matrix[0][2] = 0.4792; sqrt_corr_matrix[0][3] = -0.1204; sqrt_corr_matrix[0][4] = -0.2138;
			sqrt_corr_matrix[1][0] = 0.2450; sqrt_corr_matrix[1][1] = 0.9241; sqrt_corr_matrix[1][2] = -0.1086; sqrt_corr_matrix[1][3] = -0.2716; sqrt_corr_matrix[1][4] = 0.0218;
			sqrt_corr_matrix[2][0] = 0.4792; sqrt_corr_matrix[2][1] = -0.1086; sqrt_corr_matrix[2][2] = 0.8257; sqrt_corr_matrix[2][3] = -0.2716; sqrt_corr_matrix[2][4] = -0.0556;
			sqrt_corr_matrix[3][0] = -0.1204; sqrt_corr_matrix[3][1] = -0.2716; sqrt_corr_matrix[3][2] = -0.2716; sqrt_corr_matrix[3][3] = 0.9152; sqrt_corr_matrix[3][4] = -0.0185;
			sqrt_corr_matrix[4][0] = -0.2138; sqrt_corr_matrix[4][1] = 0.0218; sqrt_corr_matrix[4][2] = 0.0556; sqrt_corr_matrix[4][3] = -0.0185; sqrt_corr_matrix[4][4] = 0.9749;

			DS_mu = -7.03;
			DS_sigma = 0.66;
			ASD_mu = 1.15;
			ASD_sigma = 0.28;
			ASA_mu = 1.81;
			ASA_sigma = 0.20;
			SF_sigma = 4;
			K_mu = 9;
			K_sigma = 3.5;
		}
		else//���к�С���ķǿ��ӳ���
		{
			sqrt_corr_matrix[0][0] = 0.9139; sqrt_corr_matrix[0][1] = 0.1477; sqrt_corr_matrix[0][2] = 0.3180; sqrt_corr_matrix[0][3] = -0.2044; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = 0.1477; sqrt_corr_matrix[1][1] = 0.9139; sqrt_corr_matrix[1][2] = 0.2044; sqrt_corr_matrix[1][3] = -0.3180; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.3180; sqrt_corr_matrix[2][1] = 0.2044; sqrt_corr_matrix[2][2] = 0.9231; sqrt_corr_matrix[2][3] = 0.0704; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.2044; sqrt_corr_matrix[3][1] = -0.3180; sqrt_corr_matrix[3][2] = 0.0704; sqrt_corr_matrix[3][3] = 0.9231; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -6.44;
			DS_sigma = 0.39;
			ASD_mu = 1.41;
			ASD_sigma = 0.28;
			ASA_mu = 1.87;
			ASA_sigma = 0.11;
			SF_sigma = 6;
			K_mu = 0;
			K_sigma = 0;
		}
		double cross_corr_delay = 0, cross_corr_aoa = 0, cross_corr_aod = 0, cross_corr_shadow = 0, cross_corr_kfactor = 0;
		double Gaussrand_Num[5] = { 0 };
		for (int idx = 0; idx < 5; idx++)
		{
			Gaussrand_Num[idx] = RandNorm_1();
		}

		for (int idx = 0; idx < 5; idx++)
		{
			cross_corr_delay = cross_corr_delay + sqrt_corr_matrix[0][idx] * Gaussrand_Num[idx];
			cross_corr_aod = cross_corr_aod + sqrt_corr_matrix[1][idx] * Gaussrand_Num[idx];
			cross_corr_aoa = cross_corr_aoa + sqrt_corr_matrix[2][idx] * Gaussrand_Num[idx];
			cross_corr_shadow = cross_corr_shadow + sqrt_corr_matrix[3][idx] * Gaussrand_Num[idx];
			cross_corr_kfactor = cross_corr_kfactor + sqrt_corr_matrix[4][idx] * Gaussrand_Num[idx];
		}

		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->eNodeB_LSPs[index][0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);
		m_src->GetLteChannel()->eNodeB_LSPs[index][1] = min(pow(10, ASD_mu + ASD_sigma * cross_corr_aod), 104.0);//��Ϊ���Ե�λ
		m_src->GetLteChannel()->eNodeB_LSPs[index][2] = min(pow(10, ASA_mu + ASA_sigma * cross_corr_aoa), 104.0);
		m_src->GetLteChannel()->eNodeB_LSPs[index][3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
		m_src->GetLteChannel()->eNodeB_LSPs[index][4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));
		double temp = m_src->GetLteChannel()->eNodeB_LSPs[index][3];
		return 10.*log10(temp);
	}

	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
		m_isLOS = m_src->GetLteChannel()->HeNodeB_isLOS[m_dst->GetIDNetworkNode()];
		if (m_isLOS)  //����΢С���Ŀ��ӳ���
		{
			sqrt_corr_matrix[0][0] = 0.7531; sqrt_corr_matrix[0][1] = 0.2410; sqrt_corr_matrix[0][2] = 0.4541; sqrt_corr_matrix[0][3] = -0.0972; sqrt_corr_matrix[0][4] = -0.3989;
			sqrt_corr_matrix[1][0] = 0.2410; sqrt_corr_matrix[1][1] = 0.9294; sqrt_corr_matrix[1][2] = 0.1380; sqrt_corr_matrix[1][3] = -0.2424; sqrt_corr_matrix[1][4] = -0.0208;
			sqrt_corr_matrix[2][0] = 0.4541; sqrt_corr_matrix[2][1] = 0.1380; sqrt_corr_matrix[2][2] = 0.8615; sqrt_corr_matrix[2][3] = -0.1756; sqrt_corr_matrix[2][4] = -0.0414;
			sqrt_corr_matrix[3][0] = -0.0972; sqrt_corr_matrix[3][1] = -0.2424; sqrt_corr_matrix[3][2] = -0.1756; sqrt_corr_matrix[3][3] = 0.9157; sqrt_corr_matrix[3][4] = 0.2499;
			sqrt_corr_matrix[4][0] = -0.3989; sqrt_corr_matrix[4][1] = -0.0208; sqrt_corr_matrix[4][2] = -0.0414; sqrt_corr_matrix[4][3] = 0.2499; sqrt_corr_matrix[4][4] = 0.8811;

			DS_mu = -7.19;          //ʱ����չ������̬��ֵ
			DS_sigma = 0.40;        //ʱ����չ������̬��׼��
			ASD_mu = 1.2;           //�������չ
			ASD_sigma = 0.43;
			ASA_mu = 1.75;          //�������չ
			ASA_sigma = 0.19;
			SF_sigma = 3;
			K_mu = 9;               //K����
			K_sigma = 5;
		}

		else                       //����΢С���ķǿ��ӳ���
		{
			sqrt_corr_matrix[0][0] = 0.9135; sqrt_corr_matrix[0][1] = 0.0000; sqrt_corr_matrix[0][2] = 0.1780; sqrt_corr_matrix[0][3] = -0.3658; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = 0.0000; sqrt_corr_matrix[1][1] = 1.0000; sqrt_corr_matrix[1][2] = 0.0000; sqrt_corr_matrix[1][3] = 0.0000; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.1780; sqrt_corr_matrix[2][1] = 0.0000; sqrt_corr_matrix[2][2] = 0.9678; sqrt_corr_matrix[2][3] = -0.1780; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.3658; sqrt_corr_matrix[3][1] = 0.0000; sqrt_corr_matrix[3][2] = -0.1780; sqrt_corr_matrix[3][3] = 0.9135; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -6.89;
			DS_sigma = 0.54;
			ASD_mu = 1.41;
			ASD_sigma = 0.17;
			ASA_mu = 1.84;
			ASA_sigma = 0.15;
			SF_sigma = 4;
			K_mu = 0;
			K_sigma = 0;
		}
		double cross_corr_delay = 0, cross_corr_aoa = 0, cross_corr_aod = 0, cross_corr_shadow = 0, cross_corr_kfactor = 0;
		double Gaussrand_Num[5] = { 0 };
		for (int idx = 0; idx < 5; idx++)
		{
			Gaussrand_Num[idx] = RandNorm_1();
		}

		for (int idx = 0; idx < 5; idx++)
		{
			cross_corr_delay = cross_corr_delay + sqrt_corr_matrix[0][idx] * Gaussrand_Num[idx];
			cross_corr_aod = cross_corr_aod + sqrt_corr_matrix[1][idx] * Gaussrand_Num[idx];
			cross_corr_aoa = cross_corr_aoa + sqrt_corr_matrix[2][idx] * Gaussrand_Num[idx];
			cross_corr_shadow = cross_corr_shadow + sqrt_corr_matrix[3][idx] * Gaussrand_Num[idx];
			cross_corr_kfactor = cross_corr_kfactor + sqrt_corr_matrix[4][idx] * Gaussrand_Num[idx];
		}
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->HeNodeB_LSPs[index][0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][1] = min(pow(10, ASD_mu + ASD_sigma * cross_corr_aod), 104.0);//��Ϊ���Ե�λ
		m_src->GetLteChannel()->HeNodeB_LSPs[index][2] = min(pow(10, ASA_mu + ASA_sigma * cross_corr_aoa), 104.0);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));
		double temp = m_src->GetLteChannel()->HeNodeB_LSPs[index][3];
		return 10.*log10(temp);
	}

	else    // (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_UE)     //D2D
	{

		return 0;
	}
}



double
MadridGrid_PropagationLossModel::Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst)        //��������
{
	//������������
	/*
	��λ�ǣ�azimuth�������������ı����ԣ���λ���ڵ������µ�һ���Ƕȡ�����˳ʱ��Ƕȴӱ���ʼ��
	0�ȷ�λ�Ǳ�ʾ������90�ȷ�λ�Ǳ�ʾ������180�ȷ�λ�Ǳ�ʾ���ϣ�270�ȷ�λ�Ǳ�ʾ������360�ȷ�λ�Ǳ�ʾ�ǶȻع飬��Ȼ��������
	���ǣ�elevation����Ҳ�������θ߶ȡ�����λ�ǲ������֮����Ҫ���������������۲���������ڹ۲��ߵĸ߶�,����۲����ڵ����ϣ���ô���Ƿ�Χ����0�ȵ�90��֮��
	*/

	AntennaArray* antennaBs; 
	Position* Ms_pos = m_src->Getposition();
	Position* Bs_pos = m_dst->Getposition();
	double m_elevation = atan((Bs_pos->GetPositionZ() - Ms_pos->GetPositionZ()) / sqrt(pow(Ms_pos->GetPositionX() - Bs_pos->GetPositionX(), 2) + pow(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), 2))) *180. / PI;	//������� [-pi/2 , pi/2]
	double m_theta = atan2(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), Ms_pos->GetPositionX() - Bs_pos->GetPositionX())*180. / PI;  //��X����нǣ�-180��180��

	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
	{
		antennaBs= new AntennaArray(Antenna_IMTA_bs, 0., "UMa"); //�������������ڼ�������
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->thetaBS[index] = wrapAngleDistribution_1(-m_theta + 90);             //����������Ϊ�������λ�ýǶ�
		m_src->GetLteChannel()->thetaMS[index] = wrapAngleDistribution_1(-m_theta + 90 + 180.);
		m_src->GetLteChannel()->elevation[index] = m_elevation;

		if (m_theta < 0)
			m_theta = 360 + m_theta;               //��ʱ�뵽x���᷽��ļн�
		double thetaBS = wrapAngleDistribution_1(90 + 120 * (index % 3) - m_theta);                 //���ܼ����������У����������귽��Ϊ0��120��240������Ϊ0��
		return antennaBs->GetGain(thetaBS, m_elevation);
		delete antennaBs;

	}

	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
		antennaBs = new AntennaArray(Antenna_IMTA_bs, 0., "UMi"); //�������������ڼ�������
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->HethetaBS[index] = wrapAngleDistribution_1(-m_theta + 90);
		m_src->GetLteChannel()->HethetaMS[index] = wrapAngleDistribution_1(-m_theta + 90 + 180.);
		m_src->GetLteChannel()->Helevation[index] = m_elevation;

		return antennaBs->GetHeGain(m_elevation);
		delete antennaBs;
	}

	else               //(m_dst->GetNodeType() == NetworkNode::NodeType::UserEquipment)
	{
		return 0;
	}

	delete antennaBs;

}




double B1_NLOS_PL(double Dist1, double Dist2, double PL_bp, double CenterFrequency, double H_bs, double H_ms)    //Page 44 WIINERII D1.1.2V1.2
{
	double NLOS_loss = 0;
	if (0 <= Dist1&&Dist1 <= PL_bp)
		NLOS_loss = 22.7 * log10(Dist1) + 41 + 20 * log10(CenterFrequency / 5);
	else if (PL_bp <= Dist1&&Dist1 <= 5000)
		NLOS_loss = 40 * log10(Dist1) + 9.45 - 17.3 * log10(H_bs - 1) - 17.3 * log10(H_ms - 1) + 2.7 * log10(CenterFrequency / 5);

	double nj = max(2.8 - 0.0024*Dist1, 1.84);
	NLOS_loss += 20 - 12.5*nj + 10 * nj *log10(Dist2) + 3 * log10(CenterFrequency / 5); 
	return NLOS_loss;
}


void B4_NLOS_Azimuth(UserEquipment* ms, HeNodeB* bs, double * dist_out, double * dist_in, double * cos_theta)     //����С��վ�������û���������롢���ھ����Լ��Ƕȵ�
{
	/* dist_out is the distance between the outdoor terminal and the point on the wall that is nearest to the indoor interminal,
	dist_in  is the diatance from the wall to the indoor terminal
	theta is the angle between the outdoor path and the normal of the wall
	*/

	vector<Building*>* Bd = NetworkManager::Init()->GetBuildingContainer();
	std::vector<Building*>::iterator iter;
	for (iter= Bd->begin();iter!= Bd->end(); iter++)
	{
		Building* b = (*iter);
		if ((*iter)->IsInThisBuilding(ms))
		{
			Position * pos1 = b->GetCenterPosition();
			Position * pos2 = ms->Getposition();
			double Point[4][3] = { 0.0 };			//ȷ���û������ĸ�ǽ���ϵĵ�
			Point[0][0] = pos2->GetPositionX();  Point[0][1] = pos1->GetPositionY() + b->GetSide()[1]; Point[0][2] = pos2->GetPositionZ();
			Point[1][0] = pos1->GetPositionX() + b->GetSide()[0];  Point[1][1] = pos1->GetPositionY(); Point[1][2] = pos2->GetPositionZ();
			Point[2][0] = pos2->GetPositionX();  Point[2][1] = pos1->GetPositionY() - b->GetSide()[1]; Point[2][2] = pos2->GetPositionZ();
			Point[3][0] = pos1->GetPositionX() - b->GetSide()[0];  Point[3][1] = pos2->GetPositionY(); Point[3][2] = pos2->GetPositionZ();
			double DL[4] = { 0.0 };  //����
			for (int i = 0; i < 4; i++)
			{
				Position* p = new Position(Point[i][0], Point[i][1], Point[i][2]);
				DL[i] = distance(bs->Getposition(), p);
				delete p;
			}

			int flag = 0; double temp = DL[0];
			for (int i = 1; i < 4; i++)
			{
				if (DL[i] < temp)
				{
					temp = DL[i];                  //�ҵ�����ĵ�
					flag = i;
				}
			}

			(*dist_out) = DL[flag];
			Position* p = new Position(Point[flag][0], Point[flag][1], Point[flag][2]);
			(*dist_in) = distance(pos2, p);
			delete p;
           //����1
			double Vector_x1 = Point[flag][0] - pos2->GetPositionX(); 
			double Vector_y1 = Point[flag][1] - pos2->GetPositionY();
			double Vector_z1 = Point[flag][2] - pos2->GetPositionZ();
			//����2
			double Vector_x2 = bs->Getposition()->GetPositionX() - Point[flag][0]; 
			double Vector_y2 = bs->Getposition()->GetPositionY() - Point[flag][1];
			double Vector_z2 = bs->Getposition()->GetPositionZ() - Point[flag][2];

			double temp1 = Vector_x1*Vector_x2 + Vector_y1*Vector_y2 + Vector_z1*Vector_z2;
			double temp2 = sqrt(pow(Vector_x1, 2) + pow(Vector_y1, 2) + pow(Vector_z1, 2))*sqrt(pow(Vector_x2, 2) + pow(Vector_y2, 2) + pow(Vector_z2, 2));

			(*cos_theta) = temp1 / temp2;
		}

	}




}

void C4_NLOS_Azimuth(UserEquipment* ms, ENodeB* bs, double * dist_out, double * dist_in)    //winnerII C4������������վ�������û�����������ھ��� 
{

/* dist_out is the distance between the outdoor terminal and the point on the wall that is nearest to the indoor interminal,
   dist_in  is the diatance from the wall to the indoor terminal
   ���վλ�ڵ�һ����¥�Ķ��㣬������վ��������¥���û��ĵ�������������һ����¥���û�����������벻һ��
*/

	vector<Building*>* Bd = NetworkManager::Init()->GetBuildingContainer();
	std::vector<Building*>::iterator iter;
	for (iter = Bd->begin(); iter != Bd->end(); iter++)
	{
		Building* b = (*iter);

		if (b->IsInThisBuilding(ms))    //�ҵ��û�����¥��
		{
			if (b->IsInThisBuilding(bs))  //�û���վͬ¥
			{
				Position * pos1 = b->GetCenterPosition();
				Position * pos2 = ms->Getposition();
				double Point[3] = { 0.0 };			//ȷ���û�����¥���ϵĵ�
				Point[0] = pos2->GetPositionX(); Point[1] = pos2->GetPositionY(); Point[2] =(b->GetRoomheight())*(b->GetFloors());
				Position* p = new Position(Point[0], Point[1], Point[2]);
				(*dist_out) = distance(bs->Getposition(), p);
				(*dist_in) = distance(pos2, p);
				
				delete p;
			}
			else                   //�û���վ��ͬ¥
			{
				Position * pos1 = b->GetCenterPosition();
				Position * pos2 = ms->Getposition();
				double Point[4][3] = { 0.0 };			//ȷ���û������ĸ�ǽ���ϵĵ�
				Point[0][0] = pos2->GetPositionX();  Point[0][1] = pos1->GetPositionY() + b->GetSide()[1]; Point[0][2] = pos2->GetPositionZ();
				Point[1][0] = pos1->GetPositionX() + b->GetSide()[0];  Point[1][1] = pos1->GetPositionY(); Point[1][2] = pos2->GetPositionZ();
				Point[2][0] = pos2->GetPositionX();  Point[2][1] = pos1->GetPositionY() - b->GetSide()[1]; Point[2][2] = pos2->GetPositionZ();
				Point[3][0] = pos1->GetPositionX() - b->GetSide()[0];  Point[3][1] = pos2->GetPositionY(); Point[3][2] = pos2->GetPositionZ();
				double DL[4] = { 0.0 };  //����
				for (int i = 0; i < 4; i++)
				{
					Position* p = new Position(Point[i][0], Point[i][1], Point[i][2]);
					DL[i] = distance(bs->Getposition(), p);
					delete p;
				}

				int flag = 0; double temp = DL[0];
				for (int i = 1; i < 4; i++)
				{
					if (DL[i] < temp)
					{
						temp = DL[i];                  //�ҵ�����ĵ�
						flag = i;
					}
				}

				(*dist_out) = DL[flag];
				Position* p = new Position(Point[flag][0], Point[flag][1], Point[flag][2]);
				(*dist_in) = distance(pos2, p);
				delete p;
			}
		}
	}

}





double RandUni_1()
{
	return (double)(rand() + 1) / (0x7fff + 2);
}

//�������ӱ�׼��̬�ֲ��������
double RandNorm_1()
{
	double X = RandUni_1();
	double Y = RandUni_1();

	return sqrt(-2 * log(X)) * cos(2 * PI * Y);
}

double wrapAngleDistribution_1(double theta)
{
	theta = theta - 360. * floor(theta / 360.);
	return (theta - 360. * floor(theta / 180.));
}