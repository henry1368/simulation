


#include "lte-propagation-loss-model.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "LTE-A_channel.h"
#include  "Mobility.h"
#include"FFTMath.h"
#include "AntennaArray.h"
#include "parameter.h"
#include <math.h>



static double RandUni();
static double RandNorm();
static double wrapAngleDistribution(double theta);

double delay_clusters[20 + 4] = { 0.0 };//��ʼ��ʱ��
double power_path_ini[20 + 4] = { 0.0 };//��ʼ����
/*******************���վ************************/
double initial_phase_vv_LOS = 0.0;
double initial_phase_hh_LOS = 0.0;

double initial_phase_vv[20][24] = { 0.0 };
double initial_phase_vh[20][24] = { 0.0 };
double initial_phase_hv[20][24] = { 0.0 };
double initial_phase_hh[20][24] = { 0.0 };
int         num_rays[24] = { 0 };              //�洢ÿ�ص�����������ǿ���ʴشطִغ�
double arrival_angles[20][24] = { 0.0 }; //�洢���յĵ����
double depart_angles[20][24] = { 0.0 };  //�洢���յķ����
double XPR[20][24] = { 0.0 };            //�洢���漫���ȣ����Ǽ�������£�
int num_paths = 0;//һ��������Ӧһ����·���洢ÿ����·��·������


LtePropagationLossModel::LtePropagationLossModel()
{
	Fre_interval = 0.015;                                               //���ز����15khz
	numsubcarrier = int(RBs_FOR_LTE * 12);                              //ÿ��RB��12�����ز�����
	fftSize = int(pow(2, ceil(log(numsubcarrier) / log(double(2)))));   //2048 ��FFT ָ���Ͷ�������=2^����С�ڣ�log2(numsubcarrier)����; 
	sample_rate = long(fftSize* Fre_interval * 1000000);                //61.44M	
	sample_time = 1. / sample_rate;                                     //��λs 	
}

LtePropagationLossModel::~LtePropagationLossModel()
{}

void
LtePropagationLossModel::SetPropagationType(PropagationType t)
{
	m_channelType = t;
}

LtePropagationLossModel::PropagationType
LtePropagationLossModel::GetPropagationType()
{
	return m_channelType;
}


void  LtePropagationLossModel::calculateNodeB_medi(UserEquipment* m_src, vector<ENodeB*> * m_dst)
{
	eNodeB_medi.clear();
	vector<ENodeB*> ::iterator it;
	for (it = m_dst->begin(); it != m_dst->end(); it++)
	{
		ENodeB * enb = (*it);
		setEPoint(m_src, enb);
		initial();                                   //��ʼ���ŵ���ز���
		set_small_scale_parameter(SCENARIO_type);
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

		E_medi.num_paths_m = num_paths;
		E_medi.pathLoss_m = pathLoss;
		E_medi.shadowFading_m = shadowFading;
		E_medi.initial_phase_vv_LOS_m = initial_phase_vv_LOS;
		E_medi.initial_phase_hh_LOS_m = initial_phase_hh_LOS;
		for (int i = 0; i < 24; i++){ E_medi.num_rays_m[i] = num_rays[i]; }
		for (int i = 0; i < 24; i++){ E_medi.delay_clusters_m[i] = delay_clusters[i]; }
		for (int i = 0; i < 24; i++){ E_medi.power_path_ini_m[i] = power_path_ini[i]; }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++) { E_medi.initial_phase_vv_m[i][j] = initial_phase_vv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.arrival_angles_m[i][j] = arrival_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.depart_angles_m[i][j] = depart_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_vh_m[i][j] = initial_phase_vh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_hv_m[i][j] = initial_phase_hv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_hh_m[i][j] = initial_phase_hh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.XPR_m[i][j] = XPR[i][j]; } }

		eNodeB_medi.push_back(E_medi);
	}
}

void
LtePropagationLossModel::Compute_SmallscaleFading(UserEquipment* m_src, vector<ENodeB*> * m_dst,double m_time)       //С�߶�˥��
{

	    for (unsigned int index = 0; index< m_dst->size(); index++)
		{
			ENodeB* enb = m_dst->at(index);
			int id = enb->GetIDNetworkNode();

			for (int i = 0; i < RBs_FOR_LTE; i++)                                   //��ʼ��
			{
				m_src->GetLteChannel()->channel_Ecoefficients[id][i] = MatrixXcd::Zero(Nr, Nt);
			}
			time = m_time;
			K_factor = m_src->GetLteChannel()->eNodeB_LSPs[id][4];
			LOSflag = m_src->GetLteChannel()->eNodeB_isLOS[id];      //��ĳ����վ���Ӿ��־
			phi_AoD_LOS = m_src->GetLteChannel()->thetaBS[id];//�����ɴص����ʱ��Ϊ�Ӿ෽��Ӧ��
			phi_AoA_LOS = m_src->GetLteChannel()->thetaMS[id];
			travelSpeed = m_src->GetMobilityModel()->m_speed*1000. / 3600.; //��λm/s
			travelAngle = m_src->GetMobilityModel()->m_speedDirection;
			const double light_speed = 3e8;
			complex <double> channel_temp[24][Nr][Nt];            //[���ߴ���][UE������][��վ������]
			double d_u = 0;
			double d_s = 0;

			LN_size = eNodeB_medi.at(index).num_paths_m;//ÿ��������ʱ�Ӿ���	
			fc = eNodeB_medi.at(index).fc_m;
			double elevationtemp = m_src->GetLteChannel()->elevation[id];
			double Lambda = light_speed / (fc*1e9); //�ز�����
			int Secindex = id % nb_sector;
			double broadsideAzimuthBs = 60. - 120.*Secindex;        //��������˳ʱ��Ϊ����������Ű���ʱ�뷽��
			AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, broadsideAzimuthBs, SCENARIO_type); //���ɻ�վ������
			AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, broadsideAzimuthBs, SCENARIO_type); //�����û�������

			for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
			{
				//ȡδ���߶�ѹ���Ĵع���
				double K = K_factor;
				double power_n = eNodeB_medi.at(index).power_path_ini_m[clusterIdx];
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
						for (int rayIdx = 0; rayIdx < eNodeB_medi.at(index).num_rays_m[clusterIdx]; rayIdx++)
						{
							double arr_angle = eNodeB_medi.at(index).arrival_angles_m[rayIdx][clusterIdx];
							double dep_angle = eNodeB_medi.at(index).depart_angles_m[rayIdx][clusterIdx];

							double v = travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
							complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
							complex<double> F_tx_v = antennaBs->getVerticalFieldPattern(dep_angle, elevationtemp, s);
							complex <double> exp_vv(0, eNodeB_medi.at(index).initial_phase_vv_m[rayIdx][clusterIdx]);
							complex <double> j2pivt(0, 2.*PI*v*time*0.001);
							complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
							complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
							double xpr = eNodeB_medi.at(index).XPR_m[rayIdx][clusterIdx];
							complex<double> F_tx_h = antennaBs->getHorizontalFieldPattern(dep_angle, elevationtemp, s);
							complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
							complex <double> exp_vh(0, eNodeB_medi.at(index).initial_phase_vh_m[rayIdx][clusterIdx]);
							complex <double> exp_hv(0, eNodeB_medi.at(index).initial_phase_hv_m[rayIdx][clusterIdx]);
							complex <double> exp_hh(0, eNodeB_medi.at(index).initial_phase_hh_m[rayIdx][clusterIdx]);
							//��������
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								(F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / xpr)) +
								F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / xpr) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
						}

						//����������־			channel_temp[clusterIdx][u][s] *= sqrt(power_n);
						//	channel_temp[clusterIdx][u][s] *= sqrt(power_n / eNodeB_medi.at(index).num_rays_m[clusterIdx]);
						if (LOSflag == 1)
						{
							channel_temp[clusterIdx][u][s] *= sqrt(1 / (K + 1));
							if (clusterIdx == 0)
							{
								complex<double> F_rx_v_LOS = antennaMs->getominaVerticalFieldPattern(phi_AoA_LOS, elevationtemp, u);
								complex<double> F_tx_v_LOS = antennaBs->getVerticalFieldPattern(phi_AoD_LOS, elevationtemp, s);
								double v_LOS = travelSpeed*cos((phi_AoA_LOS - travelAngle)*PI / 180.) / Lambda;
								complex <double> exp_vv_LOS(0, eNodeB_medi.at(index).initial_phase_vv_LOS_m);
								complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*time*0.001);
								complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
								complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
								complex<double> F_tx_h_LOS = antennaBs->getHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
								complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
								complex <double> exp_hh_LOS(0, eNodeB_medi.at(index).initial_phase_hh_LOS_m);
								channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
									sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
							}
						}
					}
				}
			}
			double pathloss = eNodeB_medi.at(index).pathLoss_m;
			double shadow = eNodeB_medi.at(index).shadowFading_m;

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
			complex<double>  channel_Ecoefficients_freq_sub[Nr][Nt][RBs_FOR_LTE * 12] = { complex<double>(0, 0) };     //ÿһ�����ز���������Ƶ��ϵ�����м������
			complex<double>  channel_Ecoefficients_freq[Nr][Nt][RBs_FOR_LTE] = { complex<double>(0, 0) };     //ÿһ��RB��������Ƶ��ϵ�����м������
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
						k = int(eNodeB_medi.at(index).delay_clusters_m[i] / sample_time + 0.5);      //�������룬ԭ��������ȡ��
						if (k < fftSize)
						{
							fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
						}
					}

					FFTCalculation(fft_in, fft_out, fftSize);

					//fft�������Ƶ������м䣬����-pi~pi˳�����
					//����FFT֮��ȡ�㣬˳��Ϊ����ȡ600����ǰȡ600����ȥ����Ƶ
					//��ȡ600��
					for (int k = 0; k < RBs_FOR_LTE * 6; k++)
					{
						channel_Ecoefficients_freq_sub[u][s][k] = fft_out[fftSize - RBs_FOR_LTE * 6 + k];
					}
					//ǰȡ600�㣬ȥ����Ƶ
					for (int k = RBs_FOR_LTE * 6; k < RBs_FOR_LTE * 12; k++)
					{
						channel_Ecoefficients_freq_sub[u][s][k] = fft_out[k - RBs_FOR_LTE * 6 + 1];
					}

					//��1200������ȡ100������Ϊÿ��RB��Ƶ���ŵ�ϵ����Ϊ�õ�����ŵ�ϵ����Ҫ��ֵ���� 
					double unit = 1.0*(RBs_FOR_LTE * 12) / RBs_FOR_LTE;
					for (int k = 0; k < RBs_FOR_LTE; k++)
					{
						int start_spot = (int)(unit*(k + 1));
						int end_spot = start_spot + 1;
						complex<double> slope = channel_Ecoefficients_freq_sub[u][s][end_spot - 1] - channel_Ecoefficients_freq_sub[u][s][start_spot - 1];
						channel_Ecoefficients_freq[u][s][k] = (channel_Ecoefficients_freq_sub[u][s][start_spot - 1] + slope*(unit*(k + 1) - start_spot));
						//channel_Ecoefficients_freq[u][s][k] = sqrt(pathloss*shadow)*(fft_out[start_spot - 1] + slope*(unit*(k + 1) - start_spot));
					}
				}
			}



			//С�߶��ŵ�ϵ��һά�������ռ�ͳ�ʼ��
			for (int i = 0; i < RBs_FOR_LTE; i++)
			{
				complex<double> temp_array[Nt*Nr] = { 0.0 };
				for (int j = 0; j < Nt*Nr; j++)
					temp_array[j] = channel_Ecoefficients_freq[j %Nr][j / Nr][i];
				Matrix< complex<double>, Nr, Nt> B(temp_array);        //д�ɾ�����ʽ,ÿ��Ƶ���ϵ�H
				m_src->GetLteChannel()->channel_Ecoefficients[id][i] = B;
			}


			//�ͷ��ڴ�ռ�
			delete[]fft_in;
			delete[]fft_out;
			delete antennaBs;
			delete antennaMs;
		}


	
}

	
void LtePropagationLossModel::calculateHeNodeB_medi(UserEquipment* m_src, vector<HeNodeB*> * m_dst)
{
	HeNodeB_medi.clear();
	vector<HeNodeB*> ::iterator it;
	for (it = m_dst->begin(); it != m_dst->end(); it++)
	{
		HeNodeB * henb = (*it);
		setHePoint(m_src, henb);
		initial();                                   
		set_small_scale_parameter("UMi");  //��ʼ������С�߶ȵ���ز�������������µĳ������ڴ˺�������Ӹó����ŵ���С�߶Ȳ���
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

		HE_medi.fc_m = 2.5;
		HE_medi.num_paths_m = num_paths;
		HE_medi.pathLoss_m = pathLoss;
		HE_medi.shadowFading_m = shadowFading;
		HE_medi.initial_phase_vv_LOS_m = initial_phase_vv_LOS;
		HE_medi.initial_phase_hh_LOS_m = initial_phase_hh_LOS;
		for (int i = 0; i < 24; i++){ HE_medi.num_rays_m[i] = num_rays[i]; }
		for (int i = 0; i < 24; i++){ HE_medi.delay_clusters_m[i] = delay_clusters[i]; }
		for (int i = 0; i < 24; i++){ HE_medi.power_path_ini_m[i] = power_path_ini[i]; }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++) { HE_medi.initial_phase_vv_m[i][j] = initial_phase_vv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.arrival_angles_m[i][j] = arrival_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.depart_angles_m[i][j] = depart_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.initial_phase_vh_m[i][j] = initial_phase_vh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.initial_phase_hv_m[i][j] = initial_phase_hv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.initial_phase_hh_m[i][j] = initial_phase_hh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ HE_medi.XPR_m[i][j] = XPR[i][j]; } }

		HeNodeB_medi.push_back(HE_medi);
	}
}


void 
LtePropagationLossModel::Compute_SmallscaleFading(UserEquipment* m_src, vector<HeNodeB*> * m_dst, double m_time)      //С�߶�˥��
{


	for (unsigned int index = 0; index< m_dst->size(); index++)
	{
		HeNodeB* henb = m_dst->at(index);
		int id = henb->GetIDNetworkNode();

		for (int i = 0; i < RBs_FOR_LTE; i++)                                   //��ʼ��
		{
			m_src->GetLteChannel()->channel_Hecoefficients[id][i] = MatrixXcd::Zero(Nr, Nt);
		}
		time = m_time;
		travelSpeed = m_src->GetMobilityModel()->m_speed*1000. / 3600.; //��λm/s
		travelAngle = m_src->GetMobilityModel()->m_speedDirection;
		const double light_speed = 3e8;
		complex <double> channel_temp[24][Nr][Nt];            //[���ߴ���][UE������][��վ������]
		double d_u = 0;
		double d_s = 0;
		fc = HeNodeB_medi.at(index).fc_m;
		LN_size = HeNodeB_medi.at(index).num_paths_m;//ÿ��������ʱ�Ӿ���
		//�û����߿�߷��������������йأ����ڴ˴�ʹ��ȫ�����ߣ�������Ϊ0		
		double elevationtemp = m_src->GetLteChannel()->Helevation[id];
		double Lambda = light_speed / (fc*1e9); //�ز�����
		AntennaArray* antennaHeBs = new AntennaArray(Antenna_IMTA_bs, 0., SCENARIO_type);       //���ɻ�վ������
		AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, 0., SCENARIO_type);         //�����û�������

		for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
		{
			//ȡδ���߶�ѹ���Ĵع���
			double K = K_factor;
			double power_n = HeNodeB_medi.at(index).power_path_ini_m[clusterIdx];
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
					d_s = antennaHeBs->getDistanceLambda(s) * Lambda;//��վ���߼��10Lambda					
					channel_temp[clusterIdx][u][s] = (0, 0);  //[���ߴ���][UE������][��վ������]
					for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
					{
						double arr_angle = HeNodeB_medi.at(index).arrival_angles_m[rayIdx][clusterIdx];
						double dep_angle = HeNodeB_medi.at(index).depart_angles_m[rayIdx][clusterIdx];

						double v = travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
						complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
						complex<double> F_tx_v = antennaHeBs->getHeVerticalFieldPattern(dep_angle, elevationtemp, s);
						complex <double> exp_vv(0, HeNodeB_medi.at(index).initial_phase_vv_m[rayIdx][clusterIdx]);
						complex <double> j2pivt(0, 2.*PI*v*time*0.001);
						complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
						complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
						double xpr = HeNodeB_medi.at(index).XPR_m[rayIdx][clusterIdx];
						complex<double> F_tx_h = antennaHeBs->getHeHorizontalFieldPattern(dep_angle, elevationtemp, s);
						complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
						complex <double> exp_vh(0, HeNodeB_medi.at(index).initial_phase_vh_m[rayIdx][clusterIdx]);
						complex <double> exp_hv(0, HeNodeB_medi.at(index).initial_phase_hv_m[rayIdx][clusterIdx]);
						complex <double> exp_hh(0, HeNodeB_medi.at(index).initial_phase_hh_m[rayIdx][clusterIdx]);
						//��������
						channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
							(F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / xpr)) +
							F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / xpr) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
					}


					channel_temp[clusterIdx][u][s] *= sqrt(power_n);
					//					channel_temp[clusterIdx][u][s] *= sqrt(power_n / HeNodeB_medi.at(N).num_rays_m[clusterIdx]);
					if (LOSflag == 1)
					{
						channel_temp[clusterIdx][u][s] *= sqrt(1 / (K + 1));
						if (clusterIdx == 0)
						{
							complex<double> F_rx_v_LOS = antennaMs->getominaVerticalFieldPattern(phi_AoA_LOS, elevationtemp, u);
							complex<double> F_tx_v_LOS = antennaHeBs->getHeVerticalFieldPattern(phi_AoD_LOS, elevationtemp, s);
							double v_LOS = travelSpeed*cos((phi_AoA_LOS - travelAngle)*PI / 180.) / Lambda;
							complex <double> exp_vv_LOS(0, HeNodeB_medi.at(index).initial_phase_vv_LOS_m);
							complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*time*0.001);
							complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
							complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
							complex<double> F_tx_h_LOS = antennaHeBs->getHeHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
							complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
							complex <double> exp_hh_LOS(0, HeNodeB_medi.at(index).initial_phase_hh_LOS_m);
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
						}
					}
				}
			}
		}
		double pathloss = HeNodeB_medi.at(index).pathLoss_m;
		double shadow = HeNodeB_medi.at(index).shadowFading_m;

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
		/**************************************************************/
		//����Ϊ��FFT�任����ʱ���ŵ�ϵ��ת��ΪƵ���ŵ�ϵ��
		//generateFFTChannel();
		complex<double>* fft_in = new complex<double>[fftSize];
		complex<double>* fft_out = new complex<double>[fftSize];//	
		complex<double>  channel_Hecoefficients_freq_sub[Nr][Nt][RBs_FOR_LTE * 12] = { complex<double>(0, 0) };     //ÿһ�����ز���������Ƶ��ϵ�����м������
		complex<double>channel_Hecoefficients_freq[Nr][Nt][RBs_FOR_LTE] = { complex<double>(0, 0) };   //�ŵ�ϵ���м����	
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
					k = int(HeNodeB_medi.at(index).delay_clusters_m[i] / sample_time + 0.5);      //�������룬ԭ��������ȡ��
					if (k < fftSize)
					{
						fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
					}
				}

				FFTCalculation(fft_in, fft_out, fftSize);
				//����FFT֮��ȡ�㣬˳��Ϊ����ȡ600����ǰȡ600����ȥ����Ƶ
				//��ȡ600��
				for (int k = 0; k < RBs_FOR_LTE * 6; k++)
				{
					channel_Hecoefficients_freq_sub[u][s][k] = fft_out[fftSize - RBs_FOR_LTE * 6 + k];
				}
				//ǰȥ600�㣬ȥ����Ƶ
				for (int k = RBs_FOR_LTE * 6; k < RBs_FOR_LTE * 12; k++)
				{
					channel_Hecoefficients_freq_sub[u][s][k] = fft_out[k - RBs_FOR_LTE * 6 + 1];
				}

				//��1200������ȡ100������Ϊÿ��RB��Ƶ���ŵ�ϵ����Ϊ�õ�����ŵ�ϵ����Ҫ��ֵ���� 
				double unit = 1.0*(RBs_FOR_LTE * 12) / RBs_FOR_LTE;
				for (int k = 0; k < RBs_FOR_LTE; k++)
				{
					int start_spot = (int)(unit*(k + 1));
					int end_spot = start_spot + 1;
					complex<double> slope = channel_Hecoefficients_freq_sub[u][s][end_spot - 1] - channel_Hecoefficients_freq_sub[u][s][start_spot - 1];
					channel_Hecoefficients_freq[u][s][k] = (channel_Hecoefficients_freq_sub[u][s][start_spot - 1] + slope*(unit*(k + 1) - start_spot));
					//channel_Ecoefficients_freq[u][s][k] = sqrt(pathloss*shadow)*(fft_out[start_spot - 1] + slope*(unit*(k + 1) - start_spot));				}			
				}
			}
		}


		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			complex<double> temp_array[Nt*Nr] = { 0.0 };
			for (int j = 0; j < Nt*Nr; j++)
				temp_array[j] = channel_Hecoefficients_freq[j %Nr][j / Nr][i];

			Matrix< complex<double>, Nr, Nt> B(temp_array);        //д�ɾ�����ʽ,ÿ��Ƶ���ϵ�H
			m_src->GetLteChannel()->channel_Hecoefficients[id][i] = B;
		}


		//�ͷ��ڴ�ռ�
		delete[]fft_in;
		delete[]fft_out;
		delete antennaHeBs;
		delete antennaMs;
	}



}

/**********С�߶��ŵ���������*********************************************/

//С��վ��ʼ��������
void LtePropagationLossModel::setHePoint(UserEquipment* Ms, HeNodeB * Bs)
{
	Ms_index = Ms->GetIDNetworkNode();
	Bs_index = Bs->GetIDNetworkNode();
	double pathlosstemp = -(Ms->GetLteChannel()->HeNodeB_path_loss[Bs_index]);
	double shadowFadingtemp = -(Ms->GetLteChannel()->HeNodeB_shadow_loss[Bs_index]); 
	LOSflag = Ms->GetLteChannel()->HeNodeB_isLOS[Bs_index]; 
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = Ms->GetLteChannel()->HethetaBS[Bs_index]; //�����ɴص����ʱ��Ϊ�Ӿ෽��Ӧ��
	phi_AoA_LOS = Ms->GetLteChannel()->HethetaMS[Bs_index];

	delay_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][0];
	AoD_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][1];
	AoA_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][2];
	K_factor = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][4];
	K_factor_db = 10. * log10(K_factor);

}

//���վ��ʼ��������
void LtePropagationLossModel::setEPoint(UserEquipment* Ms, ENodeB * Bs)
{
	Ms_index = Ms->GetIDNetworkNode();
	Bs_index = Bs->GetIDNetworkNode();
	double pathlosstemp = -(Ms->GetLteChannel()->eNodeB_path_loss[Bs_index]);
	double shadowFadingtemp = -(Ms->GetLteChannel()->eNodeB_shadow_loss[Bs_index]);
	LOSflag = Ms->GetLteChannel()->eNodeB_isLOS[Bs_index];
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = Ms->GetLteChannel()->thetaBS[Bs_index]; //�����ɴص����ʱ��Ϊ�Ӿ෽��Ӧ��
	phi_AoA_LOS = Ms->GetLteChannel()->thetaMS[Bs_index];

	delay_spread = Ms->GetLteChannel()->eNodeB_LSPs[Bs_index][0];
	AoD_spread = Ms->GetLteChannel()->eNodeB_LSPs[Bs_index][1];
	AoA_spread = Ms->GetLteChannel()->eNodeB_LSPs[Bs_index][2];
	K_factor = Ms->GetLteChannel()->eNodeB_LSPs[Bs_index][4];
	K_factor_db = 10. * log10(K_factor);
}

void LtePropagationLossModel::initial()
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

void LtePropagationLossModel::set_small_scale_parameter(char * str)            //С�߶���ز�������
{
	if (strcmp(str, "UMi") == 0 || strcmp(str, "TC1") == 0)
	{
		if (LOSflag == 1)         //����΢С���Ŀ��Ӵ���
		{
			rtau = 3.2;             //ʱ�����Ų���
			XPR_mu = 9;             //Cross-polarization ratio  ���漫����
			XPR_sigma = 0;          // 3;          
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
		else //����΢С���ķǿ��Ӵ���
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
	}
	if ((strcmp(str, "UMa") == 0) || (strcmp(str, "TC2") == 0))//UMa��TC2
	{
		if (LOSflag == 1)   //���к�С���Ŀ��Ӵ���
		{
			rtau = 2.5;
			XPR_mu = 8;
			XPR_sigma = 0;// 4;
			clusterNum = 12;
			Caod = 5;
			Caoa = 11;
			zita = 3;
			ASD_DS = 0.4;
			ASA_DS = 0.8;
			ASA_SF = -0.5;
			ASD_SF = -0.5;
			DS_SF = -0.4;
			ASD_ASA = 0;
			ASD_K = 0;
			ASA_K = -0.2;
			DS_K = -0.4;
			SF_K = 0;

			DS_dis = 30;
			ASD_dis = 18;
			ASA_dis = 15;
			SF_dis = 37;
			K_dis = 12;
		}
		else//���к�С���ǿ��Ӵ���
		{
			rtau = 2.3;
			XPR_mu = 7;
			XPR_sigma = 0;// 3;
			clusterNum = 20;
			Caoa = 15;
			Caod = 2;
			zita = 3;
			ASD_DS = 0.4;
			ASA_DS = 0.6;
			ASA_SF = 0;
			ASD_SF = -0.6;
			DS_SF = -0.4;
			ASD_ASA = 0.4;
			ASD_K = 0;
			ASA_K = 0;
			DS_K = 0;
			SF_K = 0;

			DS_dis = 40;
			ASD_dis = 50;
			ASA_dis = 50;
			SF_dis = 50;
			K_dis = 0;
		}
	}
	if (strcmp(SCENARIO_type, "SMa") == 0)//Sma
	{
		if (LOSflag == 1)//������С���Ŀ��Ӵ���
		{
			rtau = 2.4;
			XPR_mu = 8;
			XPR_sigma = 0;// 4;//�ò��������û�� ע�������û���õ�   
			clusterNum = 15;
			Caod = 5;
			Caoa = 5;
			zita = 3;
			ASD_DS = 0;// 0.2;//0;
			ASA_DS = 0.8;
			ASA_SF = -0.5;
			ASD_SF = -0.5;
			DS_SF = -0.6;
			ASD_ASA = 0;// 0.1;// 0;
			ASD_K = 0;// 0.2;// 0;
			ASA_K = 0;// -0.2;// 0;
			DS_K = 0;// -0.2;//0;
			SF_K = 0;

			DS_dis = 6;//�����ǲ�����ؾ���
			ASD_dis = 15;
			ASA_dis = 20;
			SF_dis = 40;
			K_dis = 10;
		}
		else//������ģ�͵ķǿ��Ӵ���
		{
			rtau = 1.5;
			XPR_mu = 4;
			XPR_sigma = 0;// 3;
			clusterNum = 14;
			Caoa = 10;
			Caod = 2;
			zita = 3;
			//LOSflag=0;                   
			ASD_DS = 0;// 0.3;// 0;
			ASA_DS = 0.7;
			ASA_SF = 0;// -0.3;// 0;
			ASD_SF = -0.4;
			DS_SF = -0.4;
			ASD_ASA = 0;// 0.3;// 0;
			ASD_K = 0;
			ASA_K = 0;
			DS_K = 0;
			SF_K = 0;

			DS_dis = 40;
			ASD_dis = 30;
			ASA_dis = 30;
			SF_dis = 50;
			K_dis = 0;
		}
	}
	if (strcmp(SCENARIO_type, "RMa") == 0)//Rma
	{
		if (LOSflag == 1)//����С���Ŀ��Ӵ���
		{
			rtau = 3.8;
			XPR_mu = 12;
			XPR_sigma = 0;// 8;
			clusterNum = 11;
			Caoa = 3;
			Caod = 2;
			zita = 3;
			ASD_DS = 0;
			ASA_DS = 0;
			ASA_SF = 0;
			ASD_SF = 0;
			DS_SF = -0.5;
			ASD_ASA = 0;
			ASD_K = 0;
			ASA_K = 0;
			DS_K = 0;
			SF_K = 0;

			DS_dis = 50;
			ASD_dis = 25;
			ASA_dis = 35;
			SF_dis = 37;
			K_dis = 40;
		}
		else//����С���ķǿ��Ӵ���
		{
			rtau = 1.7;
			XPR_mu = 7;
			XPR_sigma = 0;// 4;
			clusterNum = 10;
			Caoa = 3;
			Caod = 2;
			zita = 3;
			ASD_DS = -0.4;
			ASA_DS = 0;
			ASA_SF = 0;
			ASD_SF = 0.6;
			DS_SF = -0.5;
			ASD_ASA = 0;
			ASD_K = 0;
			ASA_K = 0;
			DS_K = 0;
			SF_K = 0;

			DS_dis = 36;
			ASD_dis = 30;
			ASA_dis = 40;
			SF_dis = 120;
			K_dis = 0;
		}
	}

	clusrerNum_ini = clusterNum;    //�洢�ص�������clusterNum�ڹ��ʷִغ����л�ı�ֵ��
}

void LtePropagationLossModel::initial_Array()//�Ժ��������Ҫ��������г�ʼ�������ǵú����ͷ��ڴ�ռ�
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
void LtePropagationLossModel::generate_delay()
{
	//����ʱ��
	for (int idx = 0; idx < clusterNum; idx++)
	{
		delay_clusters[idx] = -rtau * delay_spread * log(RandUni());
	}

	//�ҵ���Сʱ��
	double delaymin = delay_clusters[0];
	for (int i = 0; i < clusterNum; i++)
	{
		delaymin = (delaymin < delay_clusters[i]) ? delaymin : delay_clusters[i];
	}
	//ʱ�ӹ�һ��
	for (int i = 0; i < clusterNum; i++)
	{
		delay_clusters[i] = delay_clusters[i] - delaymin;
	}
	//ʱ������
	double delay_temp = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		for (int j = clusterNum - 1; j > i; j--)
		{
			if (delay_clusters[j]< delay_clusters[j - 1])
			{
				delay_temp = delay_clusters[j - 1];
				delay_clusters[j - 1] = delay_clusters[j];
				delay_clusters[j] = delay_temp;
			}
		}
	}
	for (int i = 0; i < clusterNum; i++)
	{
		delay_path[i] = delay_clusters[i];
	}
}
//������   ��  generate_Gauss()
//�������ܣ����ɸ�˹������������ڹ��ʡ�����ǡ������ʱ����
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_Gauss()
{
	for (int i = 0; i < clusterNum; i++)
	{
		gaussRand[i] = RandNorm();
	}
}
//////////////////////////////////////////////////////////////////////////
//������   ��  generate_powers()
//�������ܣ������ŵ��еĹ���
//���������������ţ�������Ŀ(��վ���ն�)
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_powers()
{

	double powersum = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = exp(-delay_path[i] * (rtau - 1.) / (rtau*delay_spread))*pow(10., -gaussRand[i] * 3. / 10.);//zita�����г�����3dB��a*zita�������ֵ����׼��3dB�ĸ�˹�������
		if (power_path_ini[i] < 0)
		{
			cout << "�ع���ֵΪ������ֵΪ��" << power_path_ini[i] << endl;
			system("pause");
		}
		powersum = powersum + power_path_ini[i];
	}

	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = power_path_ini[i] / powersum;//�����ܹ��ʣ����ʹ�һ��
	}

	max_power = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		if (max_power < power_path_ini[i])
			max_power = power_path_ini[i];
	}//�ҳ������

	//�Ƴ�����С�Ĵأ��������С25 dB��
	num_paths = clusterNum;
	for (int i = 0, j = 0; i < clusterNum; i++)
	{
		if (max_power / 316.2278>power_path_ini[i])
		{
			num_paths--;
		}
		else
		{
			power_path[j] = power_path_ini[i];
			delay_path[j] = delay_clusters[i];
			j++;
		}
		power_path_ini[i] = 0;
		delay_clusters[i] = 0;//�������ƺ�û��ʲô��
	}
	clusterNum = num_paths;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = power_path[i];
		delay_clusters[i] = delay_path[i];
	}
	for (int i = 0; i < clusterNum; i++)
	{
		power_path[i] = power_path_ini[i];
	}

	//LOS�����¹��ʵ���
	if (LOSflag == 1)
	{
		power_path[0] = K_factor / (K_factor + 1) + power_path[0] / (K_factor + 1);
		power_path_ini[0] = power_path[0];
		for (int i = 1; i < clusterNum; i++)
		{
			power_path[i] = power_path[i] / (K_factor + 1);
			if (power_path[i] < 0)
			{
				cout << "LOS�´ع���ֵΪ������ֵΪ��" << power_path[i] << endl;
				system("pause");
			}
			power_path_ini[i] = power_path[i];
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
			delay_clusters[i] = delay_clusters[i] / D_factor;
			if (!_finite(abs(delay_clusters[i])))
			{
				cout << "��" << i << "���ྶ��������" << endl;
				cout << "delay_path[i]=" << delay_clusters[i] << endl;
				for (int i = 0; i < clusterNum; i++)
				{
					cout << "��" << i << "����ʱ��=" << delay_clusters[i] << endl;
				}
				cout << "K����=" << K_factor << endl;
				cout << "D����=" << D_factor << endl;
				cout << "delay_clusters�����⣡" << endl;
				system("pause");
			}
		}
	}
	for (int i = 0; i < clusterNum; i++)
	{
		delay_path[i] = delay_clusters[i];
	}

}


//////////////////////////////////////////////////////////////////////////
//������   ��generate_c()
//�������ܣ������ŵ��е�Kϵ��
//����������������
//�����������(��ȫ�ֱ������޸�)
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_c()
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
void LtePropagationLossModel::generate_AoAs()
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
		int x = 2 * (int)(2 * RandUni()) - 1;//-1��1�ľ��ȱ���
		double y = AoA_spread / 7.*RandNorm();
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
void LtePropagationLossModel::generate_AoDs()
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
		int x = 2 * (int)(2 * RandUni()) - 1;
		double y = AoD_spread / 7.*RandNorm();
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
void LtePropagationLossModel::set_SubClusters()
{
	double single_los_ray_power = 0.;
	if (LOSflag == 1)
		single_los_ray_power = K_factor / (K_factor + 1);
	power_path[0] = power_path_ini[0] - single_los_ray_power;
	for (int i = 1; i<clusterNum; i++)
	{
		power_path[i] = power_path_ini[i];
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
			num_rays[cluster_idx] = 10;//ÿ���ص�����������
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
			num_rays[cluster_idx] = 6;
			offset_angle[0][cluster_idx] = 0.5129;
			offset_angle[1][cluster_idx] = -0.5129;
			offset_angle[2][cluster_idx] = 0.6797;
			offset_angle[3][cluster_idx] = -0.6797;
			offset_angle[4][cluster_idx] = 1.5195;
			offset_angle[5][cluster_idx] = -1.5195;
		}
		else if (cluster_idx == clusterNum + 1 || cluster_idx == clusterNum + 3)
		{
			num_rays[cluster_idx] = 4;
			offset_angle[0][cluster_idx] = 0.8844;
			offset_angle[1][cluster_idx] = -0.8844;
			offset_angle[2][cluster_idx] = 1.4181;
			offset_angle[3][cluster_idx] = -1.4181;
		}
		else
		{
			num_rays[cluster_idx] = 20;
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
	num_paths += 4;//�����clusternum����һ��

	//��������ߴص�ʱ�ӡ����ʣ�ÿ�����ߵĵ���Ǻͷ���ǣ����Ʒ�Χ��
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		delay_clusters[clusterIdx] = delay_path[clusterIdx];
		if (clusterIdx == 0)
			power_path_ini[clusterIdx] = power_path[0] + single_los_ray_power;
		else
			power_path_ini[clusterIdx] = power_path[clusterIdx];
		for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
		{
			arrival_angles[rayIdx][clusterIdx] = arrival_angles_ini[clusterIdx] + Caoa*offset_angle[rayIdx][clusterIdx];
			arrival_angles[rayIdx][clusterIdx] = wrapAngleDistribution(arrival_angles[rayIdx][clusterIdx]);
			depart_angles[rayIdx][clusterIdx] = depart_angles_ini[clusterIdx] + Caod*offset_angle[rayIdx][clusterIdx];
			depart_angles[rayIdx][clusterIdx] = wrapAngleDistribution(depart_angles[rayIdx][clusterIdx]);
		}
	}

	//AOA��AOD�����ԣ�ͨ��AOA�������ʵ�֣�������ͬ���������������ÿ�����������ȡ����С�������������ΪAOA�������е�����
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		double* temp_angle = new double[num_rays[clusterIdx]];
		double* temp_rand = new double[num_rays[clusterIdx]];
		int* temp_order = new int[num_rays[clusterIdx]];
		for (int i = 0; i < num_rays[clusterIdx]; i++)
		{
			temp_angle[i] = arrival_angles[i][clusterIdx];
			temp_rand[i] = RandUni();
			temp_order[i] = 0;
		}
		for (int i = 0; i < num_rays[clusterIdx]; i++)
		{
			for (int j = 0; j < num_rays[clusterIdx]; j++)
			{
				if (temp_rand[j] < temp_rand[i])
				{
					temp_order[i]++;
				}
			}
		}
		for (int i = 0; i < num_rays[clusterIdx]; i++)
		{
			arrival_angles[i][clusterIdx] = temp_angle[temp_order[i]];
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

void LtePropagationLossModel::generate_initial_phase()
{

	initial_phase_vv_LOS = (RandUni() * 2 - 1) * PI;
	initial_phase_hh_LOS = (RandUni() * 2 - 1) * PI;
	for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
	{
		for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
		{
			initial_phase_vv[rayIdx][clusterIdx] = (RandUni() * 2 - 1) * PI;
			initial_phase_vh[rayIdx][clusterIdx] = (RandUni() * 2 - 1) * PI;
			initial_phase_hv[rayIdx][clusterIdx] = (RandUni() * 2 - 1) * PI;
			initial_phase_hh[rayIdx][clusterIdx] = (RandUni() * 2 - 1) * PI;
		}
	}

}
//////////////////////////////////////////////////////////////////////////
//������  ��generate_XPR()
//�������ܣ����ɽ��漫���ȣ����Ǽ�������£�
//�����������
//����������ޣ���ȫ�ֱ������޸ģ�
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_XPR()
{
	for (int clusterIdx = 0; clusterIdx < num_paths; clusterIdx++)
	{
		for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
		{
			XPR[rayIdx][clusterIdx] = pow(10., (XPR_mu + XPR_sigma * RandNorm()) / 10.);
		}
	}
}



double RandUni()
{
	return (double)(rand() + 1) / (RAND_MAX + 2);
}


//�������ӱ�׼��̬�ֲ��������
double RandNorm()
{
	double X = RandUni();
	double Y = RandUni();

	return sqrt(-2 * log(X)) * cos(2 * PI * Y);
}

double wrapAngleDistribution(double theta)
{
	theta = theta - 360. * floor(theta / 360.);
	return (theta - 360. * floor(theta / 180.));
}


