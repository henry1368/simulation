


#include "wifi-propagation-loss-model.h"
#include "UserEquipment.h"
#include "AP.h"
#include "WIFI_channel.h"
#include  "Mobility.h"
#include  "FFTMath.h"
#include "AntennaArray.h"
#include "parameter.h"
#include <math.h>



double RandUni_2();
double RandNorm_2();
double wrapAngleDistribution_2(double theta);

double delay_clusters_2[20 + 4] = { 0.0 };//��ʼ��ʱ��
double power_path_ini_2[20 + 4] = { 0.0 };//��ʼ����
/*******************���վ************************/
double initial_phase_vv_LOS_2 = 0.0;
double initial_phase_hh_LOS_2 = 0.0;

double iinitial_phase_vv_2[20][24] = { 0.0 };
double initial_phase_vh_2[20][24] = { 0.0 };
double initial_phase_hv_2[20][24] = { 0.0 };
double initial_phase_hh_2[20][24] = { 0.0 };
int         num_rays_2[24] = { 0 };              //�洢ÿ�ص�����������ǿ���ʴشطִغ�
double arrival_angles_2[20][24] = { 0.0 }; //�洢���յĵ����
double depart_angles_2[20][24] = { 0.0 };  //�洢���յķ����
double XPR_2[20][24] = { 0.0 };            //�洢���漫���ȣ����Ǽ�������£�
int num_paths_2 = 0;//һ��������Ӧһ����·���洢ÿ����·��·������


WifiPropagationLossModel::WifiPropagationLossModel()
{
	Fre_interval = 0.3125;       //���
	numsubcarrier = int(RBs_FOR_WIFI);      //���ز���=�ܴ���/���ز�����
	fftSize = 2 * int(pow(2, ceil(log(numsubcarrier) / log(double(2)))));		//1024��FFT����
	sample_rate = 2 * long(fftSize* Fre_interval * 1000000);//160M	
	sample_time = 1. / sample_rate;//��λs 	
	fc = 5.0;
}

WifiPropagationLossModel::~WifiPropagationLossModel()
{}

void
WifiPropagationLossModel::SetPropagationType(PropagationType t)
{
	m_channelType = t;
}

WifiPropagationLossModel::PropagationType
WifiPropagationLossModel::GetPropagationType()
{
	return m_channelType;
}




//������׽������ڳ�������TGN�ŵ�������ɲ���UMi�ŵ���WIFIĿǰ���ŵ�ģ������O2O��I2O��I2I���ǰ���UMi�ŵ�ģ�ͽ��е�
void
WifiPropagationLossModel::Compute_PowerLoss(UserEquipment* m_src, vector<AP*> *m_dst)                
{
	if (m_dst->size() > 0)     
	{

		for (unsigned int i = 0; i < m_dst->size(); i++)
		{
			AP* ap = m_dst->at(i);
			int id = ap->GetIDNetworkNode();

			//������
			m_src->GetWifiChannel()->AP_isLOS[id] = Compute_isLOS(m_src, ap);

			//·��
			m_src->GetWifiChannel()->AP_path_loss[id] = Compute_PathLoss(m_src, ap);

			//��͸���
			m_src->GetWifiChannel()->AP_penetration_loss[id] = Compute_PenetrationLoss(m_src, ap);

			//��Ӱ˥��
			m_src->GetWifiChannel()->AP_shadow_loss[id] = Compute_ShadowFading(m_src, ap);

			//��������
			m_src->GetWifiChannel()->AP_antennaGain[id] = Compute_AntennaGain(m_src, ap);

			//���
			m_src->GetWifiChannel()->AP_power_loss[id] = m_src->GetWifiChannel()->AP_antennaGain[id] - m_src->GetWifiChannel()->AP_path_loss[id]
				- m_src->GetWifiChannel()->AP_shadow_loss[id] - m_src->GetWifiChannel()->AP_penetration_loss[id] - 2;    //2dBΪ�������
		}
	}
}



bool
WifiPropagationLossModel::Compute_isLOS(UserEquipment* m_src, AP* m_dst)                //�Ӿ����
{

	/*��UMi/UMa/RMa/SMa�����У�AP���ⲿ������û��������ڻ��߳��ڣ��򲻿���
	��TC2�����У�AP���ڲ�������û�����������߳��ڣ��򲻿���
	*/
	if (m_src->GetPostate() != m_dst->GetPostate())            
		return false;
	else              
	{
		double d = distance(m_src->Getposition(), m_dst->Getposition());     //the dsitance
		double pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
		return (RandUni_2() < pLOS);
	}

}


double
WifiPropagationLossModel::Compute_PathLoss(UserEquipment* m_src, AP* m_dst) //����pathloss
{

	bool m_isLOS = m_src->GetWifiChannel()->AP_isLOS[m_dst->GetIDNetworkNode()];
	fc = 5.0;                 //carrier frequency
	this->A_medi.fc_m = fc;                                       
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
			return 0;       //����d��������Χ)       assert(false)ʱ���������еĿؼ�·��������ֵ
	}
	else         //NLOS
	{
		if (d >= 10 && d < 2000)
			return 36.7 * log10(d) + 22.7 + 26 * log10(fc);
		else
			return 0;       //����d��������Χ)       assert(false)ʱ���������еĿؼ�·��������ֵ
	}
}

double
WifiPropagationLossModel::Compute_PenetrationLoss(UserEquipment* m_src, AP* m_dst)      //��͸���
{
	if (m_dst->GetPostate() == m_Outdoor)      //AP�����⣬���ܳ���ΪUMi/MUa/RMa/SMa
	{
		if (m_src->GetPostate() == m_Indoor)      //
			return 20;

		if (m_src->GetPostate() == m_Vehicle)
			return 9 + 5.*RandNorm_2();

		else                   //(m_src->GetPostate() == Outdoor)
			return 0;
	}

	else                 //(m_dst->GetPostate() == m_Indoor)      //AP�����ڣ����ܳ���ΪTC1��TC2
	{
		if (m_src->GetPostate() == m_Indoor)
		{
		   double temp1 = m_src->Getposition()->GetPositionZ() - m_dst->Getposition()->GetPositionZ();
		   double temp2 = max(temp1, -temp1);   //��ֱ�����
		   int Nf = int(temp2 / 3.5);
		   return 17 + 4 * (Nf - 1);       //�������֮��Ĵ�͸ǽ����
		}

		else if (m_src->GetPostate() == m_Outdoor)
			return 20;

		else      //(m_src->GetPostate() == m_Vehicle)
			return 9 + 5.*RandNorm_2() + 20;

	 }
}


double
WifiPropagationLossModel::Compute_ShadowFading(UserEquipment* m_src, AP* m_dst)       //��Ӱ˥��
{
	bool m_isLOS = m_src->GetWifiChannel()->AP_isLOS[m_dst->GetIDNetworkNode()];
	double DS_mu, DS_sigma, ASD_mu, ASD_sigma, ASA_mu, ASA_sigma, SF_sigma, K_mu, K_sigma;
	double sqrt_corr_matrix[5][5] = { 0.0 };

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
		Gaussrand_Num[idx] = RandNorm_2();
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
		m_src->GetWifiChannel()->AP_LSPs[index][0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);
		m_src->GetWifiChannel()->AP_LSPs[index][1] = min(pow(10, ASD_mu + ASD_sigma * cross_corr_aod), 104.0);//��Ϊ���Ե�λ
		m_src->GetWifiChannel()->AP_LSPs[index][2] = min(pow(10, ASA_mu + ASA_sigma * cross_corr_aoa), 104.0);
		m_src->GetWifiChannel()->AP_LSPs[index][3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
		m_src->GetWifiChannel()->AP_LSPs[index][4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));
		double temp = m_src->GetWifiChannel()->AP_LSPs[index][3];
		return 10.*log10(temp);
	}






double
WifiPropagationLossModel::Compute_AntennaGain(UserEquipment* m_src, AP* m_dst)       //��������
{
	//������������
	/*
	��λ�ǣ�azimuth�������������ı����ԣ���λ���ڵ������µ�һ���Ƕȡ�����˳ʱ��Ƕȴӱ���ʼ��
	0�ȷ�λ�Ǳ�ʾ������90�ȷ�λ�Ǳ�ʾ������180�ȷ�λ�Ǳ�ʾ���ϣ�270�ȷ�λ�Ǳ�ʾ������360�ȷ�λ�Ǳ�ʾ�ǶȻع飬��Ȼ��������
	���ǣ�elevation����Ҳ�������θ߶ȡ�����λ�ǲ������֮����Ҫ���������������۲���������ڹ۲��ߵĸ߶�,����۲����ڵ����ϣ���ô���Ƿ�Χ����0�ȵ�90��֮��
	*/

	AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, 0., "UMi"); //�������������ڼ�������
	Position* Ms_pos = m_src->Getposition();
	Position* Bs_pos = m_dst->Getposition();
	double m_elevation = atan((Bs_pos->GetPositionZ() - Ms_pos->GetPositionZ()) / sqrt(pow(Ms_pos->GetPositionX() - Bs_pos->GetPositionX(), 2) + pow(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), 2))) *180. / PI;	//������� [-pi/2 , pi/2]
	double m_theta = atan2(Ms_pos->GetPositionY() - Bs_pos->GetPositionY(), Ms_pos->GetPositionX() - Bs_pos->GetPositionX())*180. / PI;  //��X����нǣ�-180��180��


	int index = m_dst->GetIDNetworkNode();
	m_src->GetWifiChannel()->APthetaBS[index] = wrapAngleDistribution_2(-m_theta + 90);             //����������Ϊ�������λ�ýǶ�
	m_src->GetWifiChannel()->APthetaMS[index] = wrapAngleDistribution_2(-m_theta + 90 + 180.);
	m_src->GetWifiChannel()->APelevation[index] = m_elevation;

   return antennaBs->GetHeGain(m_elevation);

   delete antennaBs;

}











void  WifiPropagationLossModel::calculateAP_medi(UserEquipment* m_src, vector<AP*> * m_dst)
{
	AP_medi.clear();

	vector<AP*> ::iterator it;
	for (it = m_dst->begin(); it != m_dst->end(); it++)
	{
		AP * ap = (*it);
		setEPoint(m_src, ap);
		initial();                                   //��ʼ���ŵ���ز���
		set_small_scale_parameter();
		initial_Array();
		generate_delay();
		generate_Gauss();
		generate_powers();
		generate_c();
		generate_AoAs();
		generate_AoDs();
		//ǿ���ʴ��ӳٷִء�step 8 AOA��AOD������
		set_SubClusters();
		generate_initial_phase();
		generate_XPR();

		A_medi.num_paths_m = num_paths_2;
		A_medi.pathLoss_m = pathLoss;
		A_medi.shadowFading_m = shadowFading;
		A_medi.initial_phase_vv_LOS_m = initial_phase_vv_LOS_2;
		A_medi.initial_phase_hh_LOS_m = initial_phase_hh_LOS_2;
		for (int i = 0; i < 24; i++){ A_medi.num_rays_m[i] = num_rays_2[i]; }
		for (int i = 0; i < 24; i++){ A_medi.delay_clusters_m[i] = delay_clusters_2[i]; }
		for (int i = 0; i < 24; i++){ A_medi.power_path_ini_m[i] = power_path_ini_2[i]; }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++) { A_medi.initial_phase_vv_m[i][j] = iinitial_phase_vv_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.arrival_angles_m[i][j] = arrival_angles_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.depart_angles_m[i][j] = depart_angles_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.initial_phase_vh_m[i][j] = initial_phase_vh_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.initial_phase_hv_m[i][j] = initial_phase_hv_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.initial_phase_hh_m[i][j] = initial_phase_hh_2[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ A_medi.XPR_m[i][j] = XPR_2[i][j]; } }

		AP_medi.push_back(A_medi);
	}
}

void
WifiPropagationLossModel::Compute_SmallscaleFading(UserEquipment* m_src, vector<AP*> * m_dst, double m_time)       //С�߶�˥��
{

	for (unsigned int index = 0; index< m_dst->size(); index++)
	{
		AP* ap = m_dst->at(index);
		int id = ap->GetIDNetworkNode();

		for (int i = 0; i < RBs_FOR_WIFI; i++)                                   //��ʼ��
		{
			m_src->GetWifiChannel()->channel_APcoefficients[id][i] = MatrixXcd::Zero(Nr, Nt);
		}
		time = m_time;
		K_factor = m_src->GetWifiChannel()->AP_LSPs[id][4];
		LOSflag = m_src->GetWifiChannel()->AP_isLOS[id];      //��ĳ����վ���Ӿ��־
		phi_AoD_LOS = m_src->GetWifiChannel()->APthetaBS[id];//�����ɴص����ʱ��Ϊ�Ӿ෽��Ӧ��
		phi_AoA_LOS = m_src->GetWifiChannel()->APthetaMS[id];
		travelSpeed = m_src->GetMobilityModel()->m_speed*1000. / 3600.; //��λm/s
		travelAngle = m_src->GetMobilityModel()->m_speedDirection;
		const double light_speed = 3e8;
		complex <double> channel_temp[24][Nr][Nt];            //[���ߴ���][UE������][��վ������]
		double d_u = 0;
		double d_s = 0;

		LN_size = AP_medi.at(index).num_paths_m;//ÿ��������ʱ�Ӿ���	
		fc = AP_medi.at(index).fc_m;
		double elevationtemp = m_src->GetWifiChannel()->APelevation[index];
		double Lambda = light_speed / (fc*1e9); //�ز�����
		AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, 0, "UMi"); //����AP������
		AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, 0, "UMi"); //�����û�������

		for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
		{
			//ȡδ���߶�ѹ���Ĵع���
			double K = K_factor;
			double power_n = AP_medi.at(index).power_path_ini_m[clusterIdx];
			if (LOSflag == 1)
			{
				if (clusterIdx == 0)
					power_n = power_n - K / (K + 1);
				power_n = power_n*(K + 1);
			}
			for (int u = 0; u < Nr; u++)
			{
				for (int s = 0; s < Nt; s++)
				{
					d_u = antennaMs->getDistanceLambda(u) * Lambda;//�û����߼��0.5Lambda
					d_s = antennaBs->getDistanceLambda(s) * Lambda;//��վ���߼��10Lambda					
					channel_temp[clusterIdx][u][s] = (0, 0);  //[���ߴ���][UE������][��վ������]
					for (int rayIdx = 0; rayIdx < AP_medi.at(index).num_rays_m[clusterIdx]; rayIdx++)
					{
						double arr_angle = AP_medi.at(index).arrival_angles_m[rayIdx][clusterIdx];
						double dep_angle = AP_medi.at(index).depart_angles_m[rayIdx][clusterIdx];

						double v = travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
						complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
						complex<double> F_tx_v = antennaBs->getVerticalFieldPattern(dep_angle, elevationtemp, s);
						complex <double> exp_vv(0, AP_medi.at(index).initial_phase_vv_m[rayIdx][clusterIdx]);
						complex <double> j2pivt(0, 2.*PI*v*time*0.001);
						complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
						complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
						double XPR_2 = AP_medi.at(index).XPR_m[rayIdx][clusterIdx];
						complex<double> F_tx_h = antennaBs->getHorizontalFieldPattern(dep_angle, elevationtemp, s);
						complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
						complex <double> exp_vh(0, AP_medi.at(index).initial_phase_vh_m[rayIdx][clusterIdx]);
						complex <double> exp_hv(0, AP_medi.at(index).initial_phase_hv_m[rayIdx][clusterIdx]);
						complex <double> exp_hh(0, AP_medi.at(index).initial_phase_hh_m[rayIdx][clusterIdx]);
						//��������
						channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
							(F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / XPR_2)) +
							F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / XPR_2) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
					}

					//����������־			channel_temp[clusterIdx][u][s] *= sqrt(power_n);
					//	channel_temp[clusterIdx][u][s] *= sqrt(power_n / AP_medi.at(index).num_rays_m[clusterIdx]);
					if (LOSflag == 1)
					{
						channel_temp[clusterIdx][u][s] *= sqrt(1 / (K + 1));
						if (clusterIdx == 0)
						{
							complex<double> F_rx_v_LOS = antennaMs->getominaVerticalFieldPattern(phi_AoA_LOS, elevationtemp, u);
							complex<double> F_tx_v_LOS = antennaBs->getVerticalFieldPattern(phi_AoD_LOS, elevationtemp, s);
							double v_LOS = travelSpeed*cos((phi_AoA_LOS - travelAngle)*PI / 180.) / Lambda;
							complex <double> exp_vv_LOS(0, AP_medi.at(index).initial_phase_vv_LOS_m);
							complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*time*0.001);
							complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
							complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
							complex<double> F_tx_h_LOS = antennaBs->getHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
							complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
							complex <double> exp_hh_LOS(0, AP_medi.at(index).initial_phase_hh_LOS_m);
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
						}
					}
				}
			}
		}
		double pathloss = AP_medi.at(index).pathLoss_m;
		double shadow = AP_medi.at(index).shadowFading_m;

		complex<double>  channel_coefficients_time[Nr][Nt][24] = { complex<double>(0, 0) };

		for (int j = 0; j < LN_size; j++)
		{
			for (int s = 0; s < Nt; s++)
			{
				for (int u = 0; u < Nr; u++)
				{
					channel_coefficients_time[u][s][j] = channel_temp[j][u][s];
				}
			}
		}

		//����Ϊ������Ҷ�任����ʱ���ŵ�ϵ��ת��ΪƵ���ŵ�ϵ��

		complex<double>* fft_in = new complex<double>[fftSize];
		complex<double>* fft_out = new complex<double>[fftSize];//	
		complex<double>  channel_APcoefficients_freq_sub[Nr][Nt][RBs_FOR_WIFI] = { complex<double>(0, 0) };     //ÿһ�����ز���������Ƶ��ϵ�����м������
		complex<double>  channel_APcoefficients_freq[Nr][Nt][RBs_FOR_WIFI] = { complex<double>(0, 0) };     //ÿһ��RB��������Ƶ��ϵ�����м������
		for (int u = 0; u < Nr; u++)
		{
			for (int s = 0; s < Nt; s++)
			{
				for (int j = 0; j < fftSize; j++)
				{
					fft_in[j] = complex<double>(0, 0);
					fft_out[j] = complex<double>(0, 0);
				}

				for (int i = 0; i < LN_size; i++)
				{
					int k = 0;
					k = int(AP_medi.at(index).delay_clusters_m[i] / sample_time + 0.5);      //�������룬ԭ��������ȡ��
					if (k < fftSize)
					{
						fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
					}
				}

				FFTCalculation(fft_in, fft_out, fftSize);
				//����FFT֮��ȡ�㣬˳��Ϊ����ȡ128����ǰȡ128����ȥ����Ƶ
				//��ȡ128��
				for (int k = 0; k< RBs_FOR_WIFI / 2; k++)
				{
					channel_APcoefficients_freq[u][s][k] = fft_out[fftSize - RBs_FOR_WIFI / 2 + k];
				}
				//ǰȥ128�㣬ȥ����Ƶ
				for (int k = RBs_FOR_WIFI / 2; k < RBs_FOR_WIFI; k++)
				{
					channel_APcoefficients_freq[u][s][k] = fft_out[k - RBs_FOR_WIFI / 2 + 1];
				}
			}
		}

		//С�߶��ŵ�ϵ��һά�������ռ�ͳ�ʼ��
		for (int i = 0; i < RBs_FOR_WIFI; i++)
		{
			complex<double> temp_array[Nt*Nr] = { 0.0 };
			for (int j = 0; j < Nt*Nr; j++)
				temp_array[j] = channel_APcoefficients_freq[j %Nr][j / Nr][i];

			Matrix< complex<double>, Nr, Nt> B(temp_array);        //д�ɾ�����ʽ,ÿ��Ƶ���ϵ�H				 
			m_src->GetWifiChannel()->channel_APcoefficients[id][i] = B;
		}


		//�ͷ��ڴ�ռ�
		delete[]fft_in;
		delete[]fft_out;
		delete antennaBs;
		delete antennaMs;
		}

}



/**********С�߶��ŵ���������*********************************************/

//AP��ʼ��������
void WifiPropagationLossModel::setEPoint(UserEquipment* Ms, AP * Bs)
{
	Ms_index = Ms->GetIDNetworkNode();
	Bs_index = Bs->GetIDNetworkNode();
	double pathlosstemp = -(Ms->GetWifiChannel()->AP_path_loss[Bs_index]);
	double shadowFadingtemp = -(Ms->GetWifiChannel()->AP_shadow_loss[Bs_index]);
	LOSflag = Ms->GetWifiChannel()->AP_isLOS[Bs_index];
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = Ms->GetWifiChannel()->APthetaBS[Bs_index]; //�����ɴص����ʱ��Ϊ�Ӿ෽��Ӧ��
	phi_AoA_LOS = Ms->GetWifiChannel()->APthetaMS[Bs_index];

	delay_spread = Ms->GetWifiChannel()->AP_LSPs[Bs_index][0];
	AoD_spread = Ms->GetWifiChannel()->AP_LSPs[Bs_index][1];
	AoA_spread = Ms->GetWifiChannel()->AP_LSPs[Bs_index][2];
	K_factor = Ms->GetWifiChannel()->AP_LSPs[Bs_index][4];
	K_factor_db = 10. * log10(K_factor);
}

void WifiPropagationLossModel::initial()
{
	clusterNum = 0;
	XPR_mu = 0;
	XPR_sigma = 0;
	Caod = 0;
	Caoa = 0;
	zita = 0;
	rtau = 0;
	ASD_DS = 0;
	ASA_DS = 0;
	ASA_SF = 0;
	ASD_SF = 0;
	DS_SF = 0;
	ASD_ASA = 0;
	ASD_K = 0;
	ASA_K = 0;
	DS_K = 0;
	SF_K = 0;
	DS_dis = 0;
	ASD_dis = 0;
	ASA_dis = 0;
	SF_dis = 0;
	K_dis = 0;
	max_power = 0;
}

void WifiPropagationLossModel::set_small_scale_parameter()    //WIFIĿǰ����UMi�ŵ�����
{
		if (LOSflag == 1)         //����΢С���Ŀ��Ӵ���
		{
			rtau = 3.2;             //ʱ�����Ų���
			XPR_mu = 9;             //Cross-polarization ratio  ���漫����
			XPR_sigma = 0;// 3;       
			clusterNum = 12;       //��Ҫ���������ߴ�����                   
			Caod = 3;              //���ߴط������չ
			Caoa = 17;             //���ߴص������չ
			zita = 3;              //ÿ���ߴ���Ӱ��׼��
			ASD_DS = 0.5;          //����Ϊ�����ϵ��
			ASA_DS = 0.8;
			ASA_SF = -0.4;
			ASD_SF = -0.5;
			DS_SF = -0.4;
			ASD_ASA = 0.4;
			ASD_K = -0.2;
			ASA_K = -0.3;
			DS_K = -0.7;
			SF_K = 0.5;

			DS_dis = 7;           //����Ϊ����ؾ���
			ASD_dis = 8;
			ASA_dis = 8;
			SF_dis = 10;
			K_dis = 15;
		}
		else                      //����΢С���ķǿ��Ӵ���
		{
			rtau = 3;
			XPR_mu = 8;
			XPR_sigma = 0;// 3;
			clusterNum = 19;
			Caod = 10;
			Caoa = 22;
			zita = 3;
			ASD_DS = 0;
			ASA_DS = 0.4;
			ASA_SF = -0.4;
			ASD_SF = 0;
			DS_SF = -0.7;
			ASD_ASA = 0;
			ASD_K = 0;
			ASA_K = 0;
			DS_K = 0;
			SF_K = 0;

			DS_dis = 10;
			ASD_dis = 10;
			ASA_dis = 9;
			SF_dis = 13;
			K_dis = 0;
		}

clusrerNum_ini = clusterNum;    //�洢�ص�������clusterNum�ڹ��ʷִغ����л�ı�ֵ��
}

void WifiPropagationLossModel::initial_Array()//�Ժ��������Ҫ��������г�ʼ�������ǵú����ͷ��ڴ�ռ�
{
	//����4K
	delay_path = new double[clusterNum + 4];
	power_path = new double[clusterNum + 4];
	//power_path_los = new double[clusterNum];

	arrival_angles_ini = new double[24];
	depart_angles_ini = new double[24];

	offset_angle = new double*[20];      //24*20�Ķ�̬����
	for (int i = 0; i < 20; i++)
	{
		offset_angle[i] = new double[24];
	}

	//SubClusterInd = new int[clusterNum];


}


//////////////////////////////////////////////////////////////////////////
//������   ��generate_delay()
//�������ܣ������ŵ��е��ӳ�
//���������������ǰ�û���ĳһ�������дص�ʱ�Ӿ�
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_delay()
{
	//����ʱ��
	for (int idx = 0; idx < clusterNum; idx++)
	{
		delay_clusters_2[idx] = -rtau * delay_spread * log(RandUni_2());
	}

	//�ҵ���Сʱ��
	double delaymin = delay_clusters_2[0];
	for (int i = 0; i < clusterNum; i++)
	{
		delaymin = (delaymin < delay_clusters_2[i]) ? delaymin : delay_clusters_2[i];
	}
	//ʱ�ӹ�һ��
	for (int i = 0; i < clusterNum; i++)
	{
		delay_clusters_2[i] = delay_clusters_2[i] - delaymin;
	}
	//ʱ������
	double delay_temp = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		for (int j = clusterNum - 1; j > i; j--)
		{
			if (delay_clusters_2[j]< delay_clusters_2[j - 1])
			{
				delay_temp = delay_clusters_2[j - 1];
				delay_clusters_2[j - 1] = delay_clusters_2[j];
				delay_clusters_2[j] = delay_temp;
			}
		}
	}
	for (int i = 0; i < clusterNum; i++)
	{
		delay_path[i] = delay_clusters_2[i];
	}
}
//������   ��  generate_Gauss()
//�������ܣ����ɸ�˹������������ڹ��ʡ�����ǡ������ʱ����
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_Gauss()
{
	for (int i = 0; i < clusterNum; i++)
	{
		gaussRand[i] = RandNorm_2();
	}
}
//////////////////////////////////////////////////////////////////////////
//������   ��  generate_powers()
//�������ܣ������ŵ��еĹ���
//���������������ţ�������Ŀ(��վ���ն�)
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_powers()
{

	double powersum = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini_2[i] = exp(-delay_path[i] * (rtau - 1.) / (rtau*delay_spread))*pow(10., -gaussRand[i] * 3. / 10.);//zita�����г�����3dB��a*zita�������ֵ����׼��3dB�ĸ�˹�������
		if (power_path_ini_2[i] < 0)
		{
			cout << "�ع���ֵΪ������ֵΪ��" << power_path_ini_2[i] << endl;
			system("pause");
		}
		powersum = powersum + power_path_ini_2[i];
	}

	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini_2[i] = power_path_ini_2[i] / powersum;//�����ܹ��ʣ����ʹ�һ��
	}

	max_power = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		if (max_power < power_path_ini_2[i])
			max_power = power_path_ini_2[i];
	}//�ҳ������

	//�Ƴ�����С�Ĵأ��������С25 dB��
	num_paths_2 = clusterNum;
	for (int i = 0, j = 0; i < clusterNum; i++)
	{
		if (max_power / 316.2278>power_path_ini_2[i])
		{
			num_paths_2--;
		}
		else
		{
			power_path[j] = power_path_ini_2[i];
			delay_path[j] = delay_clusters_2[i];
			j++;
		}
		power_path_ini_2[i] = 0;
		delay_clusters_2[i] = 0;//�������ƺ�û��ʲô��
	}
	clusterNum = num_paths_2;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini_2[i] = power_path[i];
		delay_clusters_2[i] = delay_path[i];
	}
	for (int i = 0; i < clusterNum; i++)
	{
		power_path[i] = power_path_ini_2[i];
	}

	//LOS�����¹��ʵ���
	if (LOSflag == 1)
	{
		power_path[0] = K_factor / (K_factor + 1) + power_path[0] / (K_factor + 1);
		power_path_ini_2[0] = power_path[0];
		for (int i = 1; i < clusterNum; i++)
		{
			power_path[i] = power_path[i] / (K_factor + 1);
			if (power_path[i] < 0)
			{
				cout << "LOS�´ع���ֵΪ������ֵΪ��" << power_path[i] << endl;
				system("pause");
			}
			power_path_ini_2[i] = power_path[i];
		}

		max_power_los = power_path[0];
		for (int i = 1; i < clusterNum; i++)
		{
			if (max_power_los < power_path[i])
				max_power_los = power_path[i];
		}
	}

	//LOS������ʱ�ӵ���
	if (LOSflag == 1)
	{
		double D_factor = 0.7705 - 0.0433*K_factor_db + 0.0002*pow(K_factor_db, 2) + 0.000017*pow(K_factor_db, 3);
		for (int i = 0; i < clusterNum; i++)
		{
			delay_clusters_2[i] = delay_clusters_2[i] / D_factor;
			if (!isfinite(abs(delay_clusters_2[i])))
			{
				cout << "��" << i << "���ྶ��������" << endl;
				cout << "delay_path[i]=" << delay_clusters_2[i] << endl;
				for (int i = 0; i < clusterNum; i++)
				{
					cout << "��" << i << "����ʱ��=" << delay_clusters_2[i] << endl;
				}
				cout << "K����=" << K_factor << endl;
				cout << "D����=" << D_factor << endl;
				cout << "delay_clusters_2�����⣡" << endl;
				system("pause");
			}
		}
	}
	for (int i = 0; i < clusterNum; i++)
	{
		delay_path[i] = delay_clusters_2[i];
	}

}



//////////////////////////////////////////////////////////////////////////
//������   ��generate_c()
//�������ܣ������ŵ��е�Kϵ��
//����������������
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_c()
{
	if (clusrerNum_ini == 4)
	{
		c_factor = 0.779;
	}
	if (clusrerNum_ini == 5)
	{
		c_factor = 0.860;
	}
	if (clusrerNum_ini == 8)
	{
		c_factor = 1.018;
	}
	if (clusrerNum_ini == 10)
	{
		c_factor = 1.090;
	}
	if (clusrerNum_ini == 11)
	{
		c_factor = 1.123;
	}
	if (clusrerNum_ini == 12)
	{
		c_factor = 1.146;
	}
	if (clusrerNum_ini == 14)
	{
		c_factor = 1.190;
	}
	if (clusrerNum_ini == 15)
	{
		c_factor = 1.211;
	}
	if (clusrerNum_ini == 16)
	{
		c_factor = 1.226;
	}
	if (clusrerNum_ini == 19)
	{
		c_factor = 1.273;
	}
	if (clusrerNum_ini == 20)
	{
		c_factor = 1.289;
	}

	if (LOSflag == 1)
	{
		c_factor = c_factor*(1.1035 - 0.028*K_factor_db - 0.002*pow(K_factor_db, 2) + 0.0001*pow(K_factor_db, 3));
	}

}
/////////////////////////////////////////////////////////////////////////
//������   generate_AoAs()
//�������ܣ���������Ƕ�
//���������������ţ���վ������Ŀ���ն�������Ŀ
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_AoAs()
{

	double first_arrival_angle;
	int X_A1;
	double Y_A1;

	for (int i = 0; i < clusterNum; i++)
	{
		if (LOSflag == 1)
		{
			arrival_angles_ini[i] = 2 * AoA_spread / 1.4*sqrt(-log(power_path[i] / max_power_los)) / c_factor;
		}
		else
		{
			arrival_angles_ini[i] = 2 * AoA_spread / 1.4*sqrt(-log(power_path[i] / max_power)) / c_factor;
		}
		int x = 2 * (int)(2 * RandUni_2()) - 1;//-1��1�ľ��ȱ���
		double y = AoA_spread / 7.*RandNorm_2();
		if (i == 0)
		{
			X_A1 = x;
			Y_A1 = y;
			first_arrival_angle = arrival_angles_ini[i];
		}
		if (LOSflag == 1)
		{
			arrival_angles_ini[i] = x * arrival_angles_ini[i] + y - (X_A1 * first_arrival_angle + Y_A1) + phi_AoA_LOS;
		}
		else
		{
			arrival_angles_ini[i] = x * arrival_angles_ini[i] + y + phi_AoA_LOS;
		}
	}
}
//////////////////////////////////////////////////////////////////////////
//������   generate_AoDs()
//�������ܣ��������ͽǶȽǶ�
//���������������ţ���վ������Ŀ���ն�������Ŀ
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_AoDs()
{

	double first_depart_angle;
	int X_D1;
	double Y_D1;

	for (int i = 0; i < clusterNum; i++)
	{
		if (LOSflag == 1)
		{
			depart_angles_ini[i] = 2 * AoD_spread / 1.4*sqrt(-log(power_path[i] / max_power_los)) / c_factor;
		}
		else
		{
			depart_angles_ini[i] = 2 * AoD_spread / 1.4*sqrt(-log(power_path[i] / max_power)) / c_factor;

		}
		int x = 2 * (int)(2 * RandUni_2()) - 1;
		double y = AoD_spread / 7.*RandNorm_2();
		if (i == 0)
		{
			X_D1 = x;
			Y_D1 = y;
			first_depart_angle = depart_angles_ini[i];
		}
		if (LOSflag == 1)
		{
			depart_angles_ini[i] = x * depart_angles_ini[i] + y - (X_D1 * first_depart_angle + Y_D1) + phi_AoD_LOS;
		}
		else
		{
			depart_angles_ini[i] = x * depart_angles_ini[i] + y + phi_AoD_LOS;
		}
	}
}
//////////////////////////////////////////////////////////////////////////
//������  ��InnerCluster_Delay()
//�������ܣ���1��2��ǿ���ʴؽ��зִش���AOA��AOD������
//�����������
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::set_SubClusters()
{
	double single_los_ray_power = 0.;
	if (LOSflag == 1)
		single_los_ray_power = K_factor / (K_factor + 1);
	power_path[0] = power_path_ini_2[0] - single_los_ray_power;
	for (int i = 1; i<clusterNum; i++)
	{
		power_path[i] = power_path_ini_2[i];
	}

	//��ȡ��ǿ2�����ʴص�����
	if (power_path[0] > power_path[1])
	{
		max_idx = 0;
		max_idx_2nd = 1;
	}
	else
	{
		max_idx = 1;
		max_idx_2nd = 0;
	}
	for (int cluster_idx = 2; cluster_idx < clusterNum; cluster_idx++)
	{
		if (power_path[cluster_idx] > power_path[max_idx_2nd])
		{
			max_idx_2nd = cluster_idx;
			if (power_path[cluster_idx] > power_path[max_idx])
			{
				int temp;
				temp = max_idx;
				max_idx = cluster_idx;
				max_idx_2nd = temp;
			}
		}
	}

	//ǿ���ʴطִغ󸽼�ʱ��
	delay_path[clusterNum] = delay_path[max_idx] + 0.000000005;
	delay_path[clusterNum + 1] = delay_path[max_idx] + 0.00000001;
	delay_path[clusterNum + 2] = delay_path[max_idx_2nd] + 0.000000005;
	delay_path[clusterNum + 3] = delay_path[max_idx_2nd] + 0.00000001;

	//ǿ���ʴطִغ�����صĹ��ʴ�С
	double power_max_idx = power_path[max_idx];                   //����ԭ����ֵ
	double power_max_idx_2nd = power_path[max_idx_2nd];     //����ԭ����ֵ

	power_path[max_idx] = power_max_idx * 10. / 20.;//ԭ�أ����ָ��ĵ�һ��
	power_path[max_idx_2nd] = power_max_idx_2nd * 10. / 20.;

	power_path[clusterNum] = power_max_idx  * 6. / 20.;//�ָ��ĵڶ���
	power_path[clusterNum + 1] = power_max_idx * 4. / 20.;

	power_path[clusterNum + 2] = power_max_idx_2nd  * 6. / 20.;
	power_path[clusterNum + 3] = power_max_idx_2nd  * 4. / 20.;

	//ǿ���ʴطִغ����AOA��AOD
	arrival_angles_ini[clusterNum] = arrival_angles_ini[max_idx];
	arrival_angles_ini[clusterNum + 1] = arrival_angles_ini[max_idx];
	arrival_angles_ini[clusterNum + 2] = arrival_angles_ini[max_idx_2nd];
	arrival_angles_ini[clusterNum + 3] = arrival_angles_ini[max_idx_2nd];

	depart_angles_ini[clusterNum] = depart_angles_ini[max_idx];
	depart_angles_ini[clusterNum + 1] = depart_angles_ini[max_idx];
	depart_angles_ini[clusterNum + 2] = depart_angles_ini[max_idx_2nd];
	depart_angles_ini[clusterNum + 3] = depart_angles_ini[max_idx_2nd];

	//���÷ִغ������ƫ�ƽǶ�
	for (int cluster_idx = 0; cluster_idx < clusterNum + 4; cluster_idx++)
	{
		if (cluster_idx == max_idx || cluster_idx == max_idx_2nd)
		{
			num_rays_2[cluster_idx] = 10;//ÿ���ص�����������
			offset_angle[0][cluster_idx] = 0.0447;
			offset_angle[1][cluster_idx] = -0.0447;
			offset_angle[2][cluster_idx] = 0.1413;
			offset_angle[3][cluster_idx] = -0.1413;
			offset_angle[4][cluster_idx] = 0.2429;
			offset_angle[5][cluster_idx] = -0.2429;
			offset_angle[6][cluster_idx] = 0.3715;
			offset_angle[7][cluster_idx] = -0.3715;
			offset_angle[8][cluster_idx] = 2.1551;
			offset_angle[9][cluster_idx] = -2.1551;
		}
		else if (cluster_idx == clusterNum || cluster_idx == clusterNum + 2)
		{
			num_rays_2[cluster_idx] = 6;
			offset_angle[0][cluster_idx] = 0.5129;
			offset_angle[1][cluster_idx] = -0.5129;
			offset_angle[2][cluster_idx] = 0.6797;
			offset_angle[3][cluster_idx] = -0.6797;
			offset_angle[4][cluster_idx] = 1.5195;
			offset_angle[5][cluster_idx] = -1.5195;
		}
		else if (cluster_idx == clusterNum + 1 || cluster_idx == clusterNum + 3)
		{
			num_rays_2[cluster_idx] = 4;
			offset_angle[0][cluster_idx] = 0.8844;
			offset_angle[1][cluster_idx] = -0.8844;
			offset_angle[2][cluster_idx] = 1.4181;
			offset_angle[3][cluster_idx] = -1.4181;
		}
		else
		{
			num_rays_2[cluster_idx] = 20;
			offset_angle[0][cluster_idx] = 0.0447;
			offset_angle[1][cluster_idx] = -0.0447;
			offset_angle[2][cluster_idx] = 0.1413;
			offset_angle[3][cluster_idx] = -0.1413;
			offset_angle[4][cluster_idx] = 0.2492;
			offset_angle[5][cluster_idx] = -0.2492;
			offset_angle[6][cluster_idx] = 0.3715;
			offset_angle[7][cluster_idx] = -0.3715;
			offset_angle[8][cluster_idx] = 0.5129;
			offset_angle[9][cluster_idx] = -0.5129;
			offset_angle[10][cluster_idx] = 0.6797;
			offset_angle[11][cluster_idx] = -0.6797;
			offset_angle[12][cluster_idx] = 0.8844;
			offset_angle[13][cluster_idx] = -0.8844;
			offset_angle[14][cluster_idx] = 1.1481;
			offset_angle[15][cluster_idx] = -1.1481;
			offset_angle[16][cluster_idx] = 1.5195;
			offset_angle[17][cluster_idx] = -1.5195;
			offset_angle[18][cluster_idx] = 2.1551;
			offset_angle[19][cluster_idx] = -2.1551;
		}
	}
	clusterNum += 4;
	num_paths_2 += 4;//�����clusternum����һ��

	//��������ߴص�ʱ�ӡ����ʣ�ÿ�����ߵĵ���Ǻͷ���ǣ����Ʒ�Χ��
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		delay_clusters_2[clusterIdx] = delay_path[clusterIdx];
		if (clusterIdx == 0)
			power_path_ini_2[clusterIdx] = power_path[0] + single_los_ray_power;
		else
			power_path_ini_2[clusterIdx] = power_path[clusterIdx];
		for (int rayIdx = 0; rayIdx < num_rays_2[clusterIdx]; rayIdx++)
		{
			arrival_angles_2[rayIdx][clusterIdx] = arrival_angles_ini[clusterIdx] + Caoa*offset_angle[rayIdx][clusterIdx];
			arrival_angles_2[rayIdx][clusterIdx] = wrapAngleDistribution_2(arrival_angles_2[rayIdx][clusterIdx]);
			depart_angles_2[rayIdx][clusterIdx] = depart_angles_ini[clusterIdx] + Caod*offset_angle[rayIdx][clusterIdx];
			depart_angles_2[rayIdx][clusterIdx] = wrapAngleDistribution_2(depart_angles_2[rayIdx][clusterIdx]);
		}
	}

	//AOA��AOD�����ԣ�ͨ��AOA�������ʵ�֣�������ͬ���������������ÿ�����������ȡ����С�������������ΪAOA�������е�����
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		double* temp_angle = new double[num_rays_2[clusterIdx]];
		double* temp_rand = new double[num_rays_2[clusterIdx]];
		int* temp_order = new int[num_rays_2[clusterIdx]];
		for (int i = 0; i < num_rays_2[clusterIdx]; i++)
		{
			temp_angle[i] = arrival_angles_2[i][clusterIdx];
			temp_rand[i] = RandUni_2();
			temp_order[i] = 0;
		}
		for (int i = 0; i < num_rays_2[clusterIdx]; i++)
		{
			for (int j = 0; j < num_rays_2[clusterIdx]; j++)
			{
				if (temp_rand[j] < temp_rand[i])
				{
					temp_order[i]++;
				}
			}
		}
		for (int i = 0; i < num_rays_2[clusterIdx]; i++)
		{
			arrival_angles_2[i][clusterIdx] = temp_angle[temp_order[i]];
		}
		delete[] temp_order;
		delete[] temp_rand;
		delete[] temp_angle;

	}
	for (int i = 0; i < 20; i++)
	{
		delete[] offset_angle[i];
	}
	delete[] offset_angle;
	delete[]delay_path;
	delete[]power_path;
	delete[] arrival_angles_ini;
	delete[] depart_angles_ini;

}

void WifiPropagationLossModel::generate_initial_phase()
{

	initial_phase_vv_LOS_2 = (RandUni_2() * 2 - 1) * PI;
	initial_phase_hh_LOS_2 = (RandUni_2() * 2 - 1) * PI;
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		for (int rayIdx = 0; rayIdx < num_rays_2[clusterIdx]; rayIdx++)
		{
			iinitial_phase_vv_2[rayIdx][clusterIdx] = (RandUni_2() * 2 - 1) * PI;
			initial_phase_vh_2[rayIdx][clusterIdx] = (RandUni_2() * 2 - 1) * PI;
			initial_phase_hv_2[rayIdx][clusterIdx] = (RandUni_2() * 2 - 1) * PI;
			initial_phase_hh_2[rayIdx][clusterIdx] = (RandUni_2() * 2 - 1) * PI;
		}
	}

}
//////////////////////////////////////////////////////////////////////////
//������  ��generate_XPR()
//�������ܣ����ɽ��漫���ȣ����Ǽ�������£�
//�����������
//����������ޣ���ȫ�ֱ������޸ģ�
//////////////////////////////////////////////////////////////////////////
void WifiPropagationLossModel::generate_XPR()
{
	for (int clusterIdx = 0; clusterIdx < num_paths_2; clusterIdx++)
	{
		for (int rayIdx = 0; rayIdx < num_rays_2[clusterIdx]; rayIdx++)
		{
			XPR_2[rayIdx][clusterIdx] = pow(10., (XPR_mu + XPR_sigma * RandNorm_2()) / 10.);
		}
	}
}



double RandUni_2()
{
	return (double)(rand() + 1) / (0x7fff + 2);
}


//�������ӱ�׼��̬�ֲ��������
double RandNorm_2()
{
	double X = RandUni_2();
	double Y = RandUni_2();

	return sqrt(-2 * log(X)) * cos(2 * PI * Y);
}

double wrapAngleDistribution_2(double theta)
{
	theta = theta - 360. * floor(theta / 360.);
	return (theta - 360. * floor(theta / 180.));
}


