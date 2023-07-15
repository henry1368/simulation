#include <cmath>
#include "urban-micro-propagation-model.h"
#include "lte-propagation-loss-model.h"
#include "NetworkNode.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "LTE-A_channel.h"
#include  "Mobility.h"
#include "AntennaArray.h"

/*
UMi场景下信道大尺度和小尺度的相关计算
*/
double RandUni_3();
double RandNorm_3();
double wrapAngleDistribution_3(double theta);

UMi_PropagationLossModel::UMi_PropagationLossModel()
{
	SetPropagationType(LtePropagationLossModel::TYPE_UMi);
}

UMi_PropagationLossModel::~UMi_PropagationLossModel()
{}

void
UMi_PropagationLossModel::Compute_PowerLoss(UserEquipment* m_src, vector<ENodeB*> *m_dst)
{
	if (m_dst->size() > 0)     //大基站
	{

		/*宏基站3个扇区之间的大尺度除了天线增益因为方位角度不一样之外，其他诸如路损、阴影、穿透损耗等均一样*/
		for (int i = 0; i < nb_cell; i++)
		{
			ENodeB* enb = m_dst->at(3 * i);
			//可视性
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i] = Compute_isLOS(m_src, enb);
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i + 1] = m_src->GetLteChannel()->eNodeB_isLOS[3 * i];
			m_src->GetLteChannel()->eNodeB_isLOS[3 * i + 2] = m_src->GetLteChannel()->eNodeB_isLOS[3 * i];

			//路损
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i] = Compute_PathLoss(m_src, enb);
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_path_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_path_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_path_loss[3 * i];

			//穿透损耗
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i] = Compute_PenetrationLoss(m_src, enb);
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_penetration_loss[3 * i];

			//阴影衰落
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i] = Compute_ShadowFading(m_src, enb);
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i + 1] = m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i];
			m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i + 2] = m_src->GetLteChannel()->eNodeB_shadow_loss[3 * i];

			//天线增益
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, enb);
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, m_dst->at(3 * i + 1));
			m_src->GetLteChannel()->eNodeB_antennaGain[3 * i] = Compute_AntennaGain(m_src, m_dst->at(3 * i + 2));       //不同扇区天线增益不同

			//LSPS参数
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
				- m_src->GetLteChannel()->eNodeB_shadow_loss[i] - m_src->GetLteChannel()->eNodeB_penetration_loss[i] - 2;    //2dB为馈线损耗
		}

	}
}



void
UMi_PropagationLossModel::Compute_PowerLoss(UserEquipment* m_src, vector<HeNodeB*> *m_dst)
{
	if (m_dst->size() > 0)     //大基站
	{

		for (unsigned int i = 0; i < m_dst->size(); i++)
		{
			HeNodeB* henb = m_dst->at(i);
			//可视性
			m_src->GetLteChannel()->HeNodeB_isLOS[i] = Compute_isLOS(m_src, henb);

			//路损
			m_src->GetLteChannel()->HeNodeB_path_loss[i] = Compute_PathLoss(m_src, henb);

			//穿透损耗
			m_src->GetLteChannel()->HeNodeB_penetration_loss[i] = Compute_PenetrationLoss(m_src, henb);

			//阴影衰落
			m_src->GetLteChannel()->HeNodeB_shadow_loss[i] = Compute_ShadowFading(m_src, henb);

			//天线增益
			m_src->GetLteChannel()->HeNodeB_antennaGain[i] = Compute_AntennaGain(m_src, henb);

			//相加
			m_src->GetLteChannel()->HeNodeB_power_loss[i] = m_src->GetLteChannel()->HeNodeB_antennaGain[i] - m_src->GetLteChannel()->HeNodeB_path_loss[i]
				- m_src->GetLteChannel()->HeNodeB_shadow_loss[i]- m_src->GetLteChannel()->HeNodeB_penetration_loss[i]- 2;    //2dB为馈线损耗
		}
	}
}



bool 
UMi_PropagationLossModel::Compute_isLOS(UserEquipment* m_src, NetworkNode* m_dst)                //视距计算
{
	//50% users outdoor(pedestrian users) and 50% of users indoors

	if (m_src->GetPostate() == m_Indoor)
		return false;
	else                    //outdoor
	{
		double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance
		double pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
		return (RandUni_3() < pLOS);                            
	}
}


double
UMi_PropagationLossModel::Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst) //计算pathloss,宏基站和小基站相同
{
	bool m_isLOS;
	if (m_dst->GetNodeType()==NetworkNode::NodeType::TYPE_ENODEB)
		m_isLOS = m_src->GetLteChannel()->eNodeB_isLOS[m_dst->GetIDNetworkNode()];
	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
		m_isLOS = m_src->GetLteChannel()->HeNodeB_isLOS[m_dst->GetIDNetworkNode()];

	 fc = 2.5;   
	 E_medi.fc_m = fc;
	 HE_medi.fc_m = fc;                                              //carrier frequency
	 const double c = 300000000;
	 double BS_HEIGHT = m_dst->Getposition()->GetPositionZ();
	 double MS_HEIGHT = m_src->Getposition()->GetPositionZ();
	 double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;   //break point distance
	 double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance
	if (m_isLOS)       //LOS         
	{
		if (d >= 10 && d < dBP)
			return  22 * log10(d) + 28 + 20 * log10(fc);
		else if (d >= dBP && d < 5000)
			return  40 * log10(d) + 7.8 - 18 * log10(BS_HEIGHT - 1) - 18 * log10(MS_HEIGHT - 1) + 2 * log10(fc);
		else
			return 0;          //距离d超出合理范围 assert(false)时：不是所有的控件路径都返回值
	}
	else         //NLOS
	{
		if (d >= 10 && d < 2000)
			return 36.7 * log10(d) + 22.7 + 26 * log10(fc);
		else
			return 0;       //距离d超出合理范围)       assert(false)时：不是所有的控件路径都返回值
    }
}

double 
UMi_PropagationLossModel::Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst)      //穿透损耗
{
	  //50% users outdoor(pedestrian users) and 50% of users indoors

	if (m_src->GetPostate() == m_Indoor)
		return   0.5 * 25 * RandUni_3() + 20;
	else
		return 0;
}


double
UMi_PropagationLossModel::Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst)       //阴影衰落
{
	bool m_isLOS;
	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
		m_isLOS = m_src->GetLteChannel()->eNodeB_isLOS[m_dst->GetIDNetworkNode()];
	else if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
		m_isLOS = m_src->GetLteChannel()->HeNodeB_isLOS[m_dst->GetIDNetworkNode()];

	double DS_mu, DS_sigma, ASD_mu, ASD_sigma, ASA_mu, ASA_sigma, SF_sigma, K_mu, K_sigma;
	double sqrt_corr_matrix[5][5] = { 0.0 };

	if (m_isLOS )  //城市微小区的可视场景
	{
		sqrt_corr_matrix[0][0] = 0.7531; sqrt_corr_matrix[0][1] = 0.2410; sqrt_corr_matrix[0][2] = 0.4541; sqrt_corr_matrix[0][3] = -0.0972; sqrt_corr_matrix[0][4] = -0.3989;
		sqrt_corr_matrix[1][0] = 0.2410; sqrt_corr_matrix[1][1] = 0.9294; sqrt_corr_matrix[1][2] = 0.1380; sqrt_corr_matrix[1][3] = -0.2424; sqrt_corr_matrix[1][4] = -0.0208;
		sqrt_corr_matrix[2][0] = 0.4541; sqrt_corr_matrix[2][1] = 0.1380; sqrt_corr_matrix[2][2] = 0.8615; sqrt_corr_matrix[2][3] = -0.1756; sqrt_corr_matrix[2][4] = -0.0414;
		sqrt_corr_matrix[3][0] = -0.0972; sqrt_corr_matrix[3][1] = -0.2424; sqrt_corr_matrix[3][2] = -0.1756; sqrt_corr_matrix[3][3] = 0.9157; sqrt_corr_matrix[3][4] = 0.2499;
		sqrt_corr_matrix[4][0] = -0.3989; sqrt_corr_matrix[4][1] = -0.0208; sqrt_corr_matrix[4][2] = -0.0414; sqrt_corr_matrix[4][3] = 0.2499; sqrt_corr_matrix[4][4] = 0.8811;

		DS_mu = -7.19;          //时延扩展对数正态均值
		DS_sigma = 0.40;        //时延扩展对数正态标准差
		ASD_mu = 1.2;           //发射角扩展
		ASD_sigma = 0.43;
		ASA_mu = 1.75;          //到达角扩展
		ASA_sigma = 0.19;
		SF_sigma = 3;
		K_mu = 9;               //K因子
		K_sigma = 5;
	}

	else                       //城市微小区的非可视场景
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
	  Gaussrand_Num[idx] = RandNorm_3();
	}

	for (int idx = 0; idx < 5; idx++)
	{
		cross_corr_delay = cross_corr_delay + sqrt_corr_matrix[0][idx] * Gaussrand_Num[idx];
		cross_corr_aod = cross_corr_aod + sqrt_corr_matrix[1][idx] * Gaussrand_Num[idx];
		cross_corr_aoa = cross_corr_aoa + sqrt_corr_matrix[2][idx] * Gaussrand_Num[idx];
		cross_corr_shadow = cross_corr_shadow + sqrt_corr_matrix[3][idx] * Gaussrand_Num[idx];
		cross_corr_kfactor = cross_corr_kfactor + sqrt_corr_matrix[4][idx] * Gaussrand_Num[idx];
	}

	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
	{
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->eNodeB_LSPs[index][0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);
		m_src->GetLteChannel()->eNodeB_LSPs[index][1] = min(pow(10, ASD_mu + ASD_sigma * cross_corr_aod),104.0);//均为线性单位
		m_src->GetLteChannel()->eNodeB_LSPs[index][2] = min(pow(10, ASA_mu + ASA_sigma * cross_corr_aoa),104.0);
		m_src->GetLteChannel()->eNodeB_LSPs[index][3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
		m_src->GetLteChannel()->eNodeB_LSPs[index][4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));
		double temp = m_src->GetLteChannel()->eNodeB_LSPs[index][3];
		return 10.*log10(temp);
	}

	
	else             //(m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->HeNodeB_LSPs[index][0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][1] = min(pow(10, ASD_mu + ASD_sigma * cross_corr_aod), 104.0);//均为线性单位
		m_src->GetLteChannel()->HeNodeB_LSPs[index][2] = min(pow(10, ASA_mu + ASA_sigma * cross_corr_aoa), 104.0);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
		m_src->GetLteChannel()->HeNodeB_LSPs[index][4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));
		double temp = m_src->GetLteChannel()->HeNodeB_LSPs[index][3];
		return 10.*log10(temp);
	}

	

}


double
UMi_PropagationLossModel::Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst)        //天线增益
{
	//计算天线增益
	/*
	方位角（azimuth），相对于物理的北而言，方位角在地面以下的一个角度。按照顺时针角度从北开始，
	0度方位角表示正北，90度方位角表示正东，180度方位角表示正南，270度方位角表示正西，360度方位角表示角度回归，依然是正北。
	仰角（elevation），也叫做海拔高度。当方位角测量完毕之后，需要用仰角来描述被观察物体相对于观察者的高度,如果观察者在地面上，那么仰角范围就在0度到90度之间
	*/

	AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, 0.,"UMi"); //生成天线类用于计算增益
	Position* Ms_pos = m_src->Getposition();
	Position* Bs_pos = m_dst->Getposition();
	double m_elevation = atan((Bs_pos->GetPositionZ() - Ms_pos->GetPositionZ()) / sqrt(pow(Ms_pos->GetPositionX() - Bs_pos->GetPositionX(), 2) + pow(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), 2))) *180. / PI;	//求出仰角 [-pi/2 , pi/2]
	double m_theta = atan2(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), Ms_pos->GetPositionX() - Bs_pos->GetPositionX())*180. / PI;  //与X正轴夹角（-180，180）

	if (m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_ENODEB)
	{
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->thetaBS[index] = wrapAngleDistribution_3(-m_theta + 90);             //以正北方向为正方向的位置角度
		m_src->GetLteChannel()->thetaMS[index] = wrapAngleDistribution_3(-m_theta + 90 + 180.);
		m_src->GetLteChannel()->elevation[index] = m_elevation;

		if (m_theta < 0)
			m_theta = 360 + m_theta;                //逆时针到x正轴方向的夹角
		double thetaBS = wrapAngleDistribution_3(30 + 120 * (index%3) - m_theta);                 //在六边形小区网络中，扇区的主瓣方向为30，150，270
		return antennaBs->GetGain(thetaBS, m_elevation);

	}

	else                   //(m_dst->GetNodeType() == NetworkNode::NodeType::TYPE_HENODEB)
	{
		int index = m_dst->GetIDNetworkNode();
		m_src->GetLteChannel()->HethetaBS[index] = wrapAngleDistribution_3(-m_theta + 90);
		m_src->GetLteChannel()->HethetaMS[index] = wrapAngleDistribution_3(-m_theta + 90 + 180.);
		m_src->GetLteChannel()->Helevation[index] = m_elevation;

		return antennaBs->GetHeGain(m_elevation);

	}
	delete antennaBs;

}





double RandUni_3()
{
	return (double)(rand() + 1) / (0x7fff + 2);
}

//产生服从标准正态分布的随机数
double RandNorm_3()
{
	double X = RandUni_3();
	double Y = RandUni_3();

	return sqrt(-2 * log(X)) * cos(2 * PI * Y);
}

double wrapAngleDistribution_3(double theta)
{
	theta = theta - 360. * floor(theta / 360.);
	return (theta - 360. * floor(theta / 180.));
}