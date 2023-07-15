


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

double delay_clusters[20 + 4] = { 0.0 };//初始延时径
double power_path_ini[20 + 4] = { 0.0 };//初始功率
/*******************大基站************************/
double initial_phase_vv_LOS = 0.0;
double initial_phase_hh_LOS = 0.0;

double initial_phase_vv[20][24] = { 0.0 };
double initial_phase_vh[20][24] = { 0.0 };
double initial_phase_hv[20][24] = { 0.0 };
double initial_phase_hh[20][24] = { 0.0 };
int         num_rays[24] = { 0 };              //存储每簇的射线数量（强功率簇簇分簇后）
double arrival_angles[20][24] = { 0.0 }; //存储最终的到达角
double depart_angles[20][24] = { 0.0 };  //存储最终的发射角
double XPR[20][24] = { 0.0 };            //存储交叉极化比（考虑极化情况下）
int num_paths = 0;//一个扇区对应一条链路，存储每条链路的路径数量


LtePropagationLossModel::LtePropagationLossModel()
{
	Fre_interval = 0.015;                                               //子载波间隔15khz
	numsubcarrier = int(RBs_FOR_LTE * 12);                              //每个RB由12个子载波构成
	fftSize = int(pow(2, ceil(log(numsubcarrier) / log(double(2)))));   //2048 点FFT 指数和对数函数=2^【不小于（log2(numsubcarrier)）】; 
	sample_rate = long(fftSize* Fre_interval * 1000000);                //61.44M	
	sample_time = 1. / sample_rate;                                     //单位s 	
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
		initial();                                   //初始化信道相关参量
		set_small_scale_parameter(SCENARIO_type);
		initial_Array();
		generate_delay();
		generate_Gauss();
		generate_powers();
		generate_c();
		generate_AoAs();
		generate_AoDs();
		//强功率簇延迟分簇、step 8 AOA与AOD随机配对
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
LtePropagationLossModel::Compute_SmallscaleFading(UserEquipment* m_src, vector<ENodeB*> * m_dst,double m_time)       //小尺度衰落
{

	    for (unsigned int index = 0; index< m_dst->size(); index++)
		{
			ENodeB* enb = m_dst->at(index);
			int id = enb->GetIDNetworkNode();

			for (int i = 0; i < RBs_FOR_LTE; i++)                                   //初始化
			{
				m_src->GetLteChannel()->channel_Ecoefficients[id][i] = MatrixXcd::Zero(Nr, Nt);
			}
			time = m_time;
			K_factor = m_src->GetLteChannel()->eNodeB_LSPs[id][4];
			LOSflag = m_src->GetLteChannel()->eNodeB_isLOS[id];      //到某个基站的视距标志
			phi_AoD_LOS = m_src->GetLteChannel()->thetaBS[id];//在生成簇到达角时作为视距方向应用
			phi_AoA_LOS = m_src->GetLteChannel()->thetaMS[id];
			travelSpeed = m_src->GetMobilityModel()->m_speed*1000. / 3600.; //单位m/s
			travelAngle = m_src->GetMobilityModel()->m_speedDirection;
			const double light_speed = 3e8;
			complex <double> channel_temp[24][Nr][Nt];            //[射线簇数][UE天线数][基站天线数]
			double d_u = 0;
			double d_s = 0;

			LN_size = eNodeB_medi.at(index).num_paths_m;//每个扇区的时延径数	
			fc = eNodeB_medi.at(index).fc_m;
			double elevationtemp = m_src->GetLteChannel()->elevation[id];
			double Lambda = light_speed / (fc*1e9); //载波波长
			int Secindex = id % nb_sector;
			double broadsideAzimuthBs = 60. - 120.*Secindex;        //正北方向顺时针为正（扇区标号按逆时针方向）
			AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, broadsideAzimuthBs, SCENARIO_type); //生成基站天线类
			AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, broadsideAzimuthBs, SCENARIO_type); //生成用户天线类

			for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
			{
				//取未经尺度压缩的簇功率
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
						d_u = antennaMs->getDistanceLambda(u) * Lambda;//用户天线间距0.5Lambda
						d_s = antennaBs->getDistanceLambda(s) * Lambda;//基站天线间距10Lambda					
						channel_temp[clusterIdx][u][s] = (0, 0);  //[射线簇数][UE天线数][基站天线数]
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
							//矩阵运算
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								(F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / xpr)) +
								F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / xpr) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
						}

						//这里做个标志			channel_temp[clusterIdx][u][s] *= sqrt(power_n);
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

			//以下为做傅里叶变换，把时域信道系数转换为频域信道系数

			complex<double>* fft_in = new complex<double>[fftSize];
			complex<double>* fft_out = new complex<double>[fftSize];//	
			complex<double>  channel_Ecoefficients_freq_sub[Nr][Nt][RBs_FOR_LTE * 12] = { complex<double>(0, 0) };     //每一个子载波到扇区的频域系数（中间变量）
			complex<double>  channel_Ecoefficients_freq[Nr][Nt][RBs_FOR_LTE] = { complex<double>(0, 0) };     //每一个RB到扇区的频域系数（中间变量）
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
						k = int(eNodeB_medi.at(index).delay_clusters_m[i] / sample_time + 0.5);      //四舍五入，原来是向下取整
						if (k < fftSize)
						{
							fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
						}
					}

					FFTCalculation(fft_in, fft_out, fftSize);

					//fft做完后将零频点调至中间，即按-pi~pi顺序出点
					//做完FFT之后取点，顺序为：后取600个，前取600个，去掉零频
					//后取600点
					for (int k = 0; k < RBs_FOR_LTE * 6; k++)
					{
						channel_Ecoefficients_freq_sub[u][s][k] = fft_out[fftSize - RBs_FOR_LTE * 6 + k];
					}
					//前取600点，去掉零频
					for (int k = RBs_FOR_LTE * 6; k < RBs_FOR_LTE * 12; k++)
					{
						channel_Ecoefficients_freq_sub[u][s][k] = fft_out[k - RBs_FOR_LTE * 6 + 1];
					}

					//从1200个点中取100个点作为每个RB的频域信道系数，为得到最后信道系数需要插值计算 
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



			//小尺度信道系数一维数组分配空间和初始化
			for (int i = 0; i < RBs_FOR_LTE; i++)
			{
				complex<double> temp_array[Nt*Nr] = { 0.0 };
				for (int j = 0; j < Nt*Nr; j++)
					temp_array[j] = channel_Ecoefficients_freq[j %Nr][j / Nr][i];
				Matrix< complex<double>, Nr, Nt> B(temp_array);        //写成矩阵形式,每个频点上的H
				m_src->GetLteChannel()->channel_Ecoefficients[id][i] = B;
			}


			//释放内存空间
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
		set_small_scale_parameter("UMi");  //初始化计算小尺度的相关参量，如若添加新的场景则在此函数中添加该场景信道的小尺度参数
		initial_Array();
		generate_delay();
		generate_Gauss();
		generate_powers();
		generate_c();
		generate_AoAs();
		generate_AoDs();
		//强功率簇延迟分簇、step 8 AOA与AOD随机配对
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
LtePropagationLossModel::Compute_SmallscaleFading(UserEquipment* m_src, vector<HeNodeB*> * m_dst, double m_time)      //小尺度衰落
{


	for (unsigned int index = 0; index< m_dst->size(); index++)
	{
		HeNodeB* henb = m_dst->at(index);
		int id = henb->GetIDNetworkNode();

		for (int i = 0; i < RBs_FOR_LTE; i++)                                   //初始化
		{
			m_src->GetLteChannel()->channel_Hecoefficients[id][i] = MatrixXcd::Zero(Nr, Nt);
		}
		time = m_time;
		travelSpeed = m_src->GetMobilityModel()->m_speed*1000. / 3600.; //单位m/s
		travelAngle = m_src->GetMobilityModel()->m_speedDirection;
		const double light_speed = 3e8;
		complex <double> channel_temp[24][Nr][Nt];            //[射线簇数][UE天线数][基站天线数]
		double d_u = 0;
		double d_s = 0;
		fc = HeNodeB_medi.at(index).fc_m;
		LN_size = HeNodeB_medi.at(index).num_paths_m;//每个扇区的时延径数
		//用户天线宽边方向，与天线类型有关，由于此处使用全向天线，故设置为0		
		double elevationtemp = m_src->GetLteChannel()->Helevation[id];
		double Lambda = light_speed / (fc*1e9); //载波波长
		AntennaArray* antennaHeBs = new AntennaArray(Antenna_IMTA_bs, 0., SCENARIO_type);       //生成基站天线类
		AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, 0., SCENARIO_type);         //生成用户天线类

		for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
		{
			//取未经尺度压缩的簇功率
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
					d_u = antennaMs->getDistanceLambda(u) * Lambda;//用户天线间距0.5Lambda
					d_s = antennaHeBs->getDistanceLambda(s) * Lambda;//基站天线间距10Lambda					
					channel_temp[clusterIdx][u][s] = (0, 0);  //[射线簇数][UE天线数][基站天线数]
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
						//矩阵运算
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
		//以下为做FFT变换，把时域信道系数转换为频域信道系数
		//generateFFTChannel();
		complex<double>* fft_in = new complex<double>[fftSize];
		complex<double>* fft_out = new complex<double>[fftSize];//	
		complex<double>  channel_Hecoefficients_freq_sub[Nr][Nt][RBs_FOR_LTE * 12] = { complex<double>(0, 0) };     //每一个子载波到扇区的频域系数（中间变量）
		complex<double>channel_Hecoefficients_freq[Nr][Nt][RBs_FOR_LTE] = { complex<double>(0, 0) };   //信道系数中间变量	
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
					k = int(HeNodeB_medi.at(index).delay_clusters_m[i] / sample_time + 0.5);      //四舍五入，原来是向下取整
					if (k < fftSize)
					{
						fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
					}
				}

				FFTCalculation(fft_in, fft_out, fftSize);
				//做完FFT之后取点，顺序为：后取600个，前取600个，去掉零频
				//后取600点
				for (int k = 0; k < RBs_FOR_LTE * 6; k++)
				{
					channel_Hecoefficients_freq_sub[u][s][k] = fft_out[fftSize - RBs_FOR_LTE * 6 + k];
				}
				//前去600点，去掉零频
				for (int k = RBs_FOR_LTE * 6; k < RBs_FOR_LTE * 12; k++)
				{
					channel_Hecoefficients_freq_sub[u][s][k] = fft_out[k - RBs_FOR_LTE * 6 + 1];
				}

				//从1200个点中取100个点作为每个RB的频域信道系数，为得到最后信道系数需要插值计算 
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

			Matrix< complex<double>, Nr, Nt> B(temp_array);        //写成矩阵形式,每个频点上的H
			m_src->GetLteChannel()->channel_Hecoefficients[id][i] = B;
		}


		//释放内存空间
		delete[]fft_in;
		delete[]fft_out;
		delete antennaHeBs;
		delete antennaMs;
	}



}

/**********小尺度信道产生程序*********************************************/

//小基站初始参数设置
void LtePropagationLossModel::setHePoint(UserEquipment* Ms, HeNodeB * Bs)
{
	Ms_index = Ms->GetIDNetworkNode();
	Bs_index = Bs->GetIDNetworkNode();
	double pathlosstemp = -(Ms->GetLteChannel()->HeNodeB_path_loss[Bs_index]);
	double shadowFadingtemp = -(Ms->GetLteChannel()->HeNodeB_shadow_loss[Bs_index]); 
	LOSflag = Ms->GetLteChannel()->HeNodeB_isLOS[Bs_index]; 
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = Ms->GetLteChannel()->HethetaBS[Bs_index]; //在生成簇到达角时作为视距方向应用
	phi_AoA_LOS = Ms->GetLteChannel()->HethetaMS[Bs_index];

	delay_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][0];
	AoD_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][1];
	AoA_spread = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][2];
	K_factor = Ms->GetLteChannel()->HeNodeB_LSPs[Bs_index][4];
	K_factor_db = 10. * log10(K_factor);

}

//大基站初始参数设置
void LtePropagationLossModel::setEPoint(UserEquipment* Ms, ENodeB * Bs)
{
	Ms_index = Ms->GetIDNetworkNode();
	Bs_index = Bs->GetIDNetworkNode();
	double pathlosstemp = -(Ms->GetLteChannel()->eNodeB_path_loss[Bs_index]);
	double shadowFadingtemp = -(Ms->GetLteChannel()->eNodeB_shadow_loss[Bs_index]);
	LOSflag = Ms->GetLteChannel()->eNodeB_isLOS[Bs_index];
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = Ms->GetLteChannel()->thetaBS[Bs_index]; //在生成簇到达角时作为视距方向应用
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

void LtePropagationLossModel::set_small_scale_parameter(char * str)            //小尺度相关参数配置
{
	if (strcmp(str, "UMi") == 0 || strcmp(str, "TC1") == 0)
	{
		if (LOSflag == 1)         //城市微小区的可视传输
		{
			rtau = 3.2;             //时延缩放参数
			XPR_mu = 9;             //Cross-polarization ratio  交叉极化比
			XPR_sigma = 0;          // 3;          
			clusterNum = 12;       //重要参量，射线簇数量                   
			Caod = 3;              //射线簇发射角扩展
			Caoa = 17;             //射线簇到达角扩展
			zita = 3;              //每射线簇阴影标准差
			ASD_DS = 0.5;          //以下为互相关系数
			ASA_DS = 0.8;
			ASA_SF = -0.4;
			ASD_SF = -0.5;
			DS_SF = -0.4;
			ASD_ASA = 0.4;
			ASD_K = -0.2;
			ASA_K = -0.3;
			DS_K = -0.7;
			SF_K = 0.5;

			DS_dis = 7;           //以下为互相关距离
			ASD_dis = 8;
			ASA_dis = 8;
			SF_dis = 10;
			K_dis = 15;
		}
		else //城市微小区的非可视传输
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
	if ((strcmp(str, "UMa") == 0) || (strcmp(str, "TC2") == 0))//UMa或TC2
	{
		if (LOSflag == 1)   //城市宏小区的可视传输
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
		else//城市宏小区非可视传输
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
		if (LOSflag == 1)//郊区宏小区的可视传输
		{
			rtau = 2.4;
			XPR_mu = 8;
			XPR_sigma = 0;// 4;//该参量表格中没有 注意后续有没有用到   
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

			DS_dis = 6;//以下是参量相关距离
			ASD_dis = 15;
			ASA_dis = 20;
			SF_dis = 40;
			K_dis = 10;
		}
		else//郊区宏模型的非可视传输
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
		if (LOSflag == 1)//乡村宏小区的可视传输
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
		else//乡村宏小区的非可视传输
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

	clusrerNum_ini = clusterNum;    //存储簇的数量（clusterNum在功率分簇函数中会改变值）
}

void LtePropagationLossModel::initial_Array()//对后面计算需要的数组进行初始化。最后记得函数释放内存空间
{
	//增长4K
	delay_path = new double[clusterNum + 4];
	power_path = new double[clusterNum + 4];
	//power_path_los = new double[clusterNum];

	arrival_angles_ini = new double[24];
	depart_angles_ini = new double[24];

	offset_angle = new double*[20];      //24*20的动态数组
	for (int i = 0; i < 20; i++)
	{
		offset_angle[i] = new double[24];
	}

	//SubClusterInd = new int[clusterNum];


}


//////////////////////////////////////////////////////////////////////////
//函数名   ：generate_delay()
//函数功能：产生信道中的延迟
//输入参数：产生当前用户到某一扇区所有簇的时延径
//输出参数：无(对全局变量的修改)
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_delay()
{
	//生成时延
	for (int idx = 0; idx < clusterNum; idx++)
	{
		delay_clusters[idx] = -rtau * delay_spread * log(RandUni());
	}

	//找到最小时延
	double delaymin = delay_clusters[0];
	for (int i = 0; i < clusterNum; i++)
	{
		delaymin = (delaymin < delay_clusters[i]) ? delaymin : delay_clusters[i];
	}
	//时延归一化
	for (int i = 0; i < clusterNum; i++)
	{
		delay_clusters[i] = delay_clusters[i] - delaymin;
	}
	//时延排序
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
//函数名   ：  generate_Gauss()
//函数功能：生成高斯随机变量，用于功率、到达角、发射角时所需
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_Gauss()
{
	for (int i = 0; i < clusterNum; i++)
	{
		gaussRand[i] = RandNorm();
	}
}
//////////////////////////////////////////////////////////////////////////
//函数名   ：  generate_powers()
//函数功能：产生信道中的功率
//输入参数：场景标号，天线数目(基站和终端)
//输出参数：无(对全局变量的修改)
//////////////////////////////////////////////////////////////////////////
void LtePropagationLossModel::generate_powers()
{

	double powersum = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = exp(-delay_path[i] * (rtau - 1.) / (rtau*delay_spread))*pow(10., -gaussRand[i] * 3. / 10.);//zita在所有场景是3dB，a*zita代表零均值，标准差3dB的高斯随机变量
		if (power_path_ini[i] < 0)
		{
			cout << "簇功率值为负数！值为：" << power_path_ini[i] << endl;
			system("pause");
		}
		powersum = powersum + power_path_ini[i];
	}

	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = power_path_ini[i] / powersum;//除以总功率，功率归一化
	}

	max_power = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		if (max_power < power_path_ini[i])
			max_power = power_path_ini[i];
	}//找出最大功率

	//移除功率小的簇（比最大功率小25 dB）
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
		delay_clusters[i] = 0;//这两句似乎没有什么用
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

	//LOS条件下功率调整
	if (LOSflag == 1)
	{
		power_path[0] = K_factor / (K_factor + 1) + power_path[0] / (K_factor + 1);
		power_path_ini[0] = power_path[0];
		for (int i = 1; i < clusterNum; i++)
		{
			power_path[i] = power_path[i] / (K_factor + 1);
			if (power_path[i] < 0)
			{
				cout << "LOS下簇功率值为负数！值为：" << power_path[i] << endl;
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

	//LOS条件下时延调整
	if (LOSflag == 1)
	{
		double D_factor = 0.7705 - 0.0433*K_factor_db + 0.0002*pow(K_factor_db, 2) + 0.000017*pow(K_factor_db, 3);
		for (int i = 0; i < clusterNum; i++)
		{
			delay_clusters[i] = delay_clusters[i] / D_factor;
			if (!_finite(abs(delay_clusters[i])))
			{
				cout << "第" << i << "条多径出现问题" << endl;
				cout << "delay_path[i]=" << delay_clusters[i] << endl;
				for (int i = 0; i < clusterNum; i++)
				{
					cout << "第" << i << "条径时延=" << delay_clusters[i] << endl;
				}
				cout << "K因子=" << K_factor << endl;
				cout << "D因子=" << D_factor << endl;
				cout << "delay_clusters有问题！" << endl;
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
//函数名   ：generate_c()
//函数功能：产生信道中的K系数
//输入参数：场景标号
//输出参数：无(对全局变量的修改)
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
//函数名   generate_AoAs()
//函数功能：产生到达角度
//输入参数：场景标号，基站天线数目，终端天线数目
//输出参数：无(对全局变量的修改)
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
		int x = 2 * (int)(2 * RandUni()) - 1;//-1到1的均匀变量
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
//函数名   generate_AoDs()
//函数功能：产生发送角度角度
//输入参数：场景标号，基站天线数目，终端天线数目
//输出参数：无(对全局变量的修改)
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
//函数名  ：InnerCluster_Delay()
//函数功能：对1，2的强功率簇进行分簇处理，AOA与AOD随机配对
//输入参数：无
//输出参数：无(对全局变量的修改)
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

	//获取最强2个功率簇的索引
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

	//强功率簇分簇后附加时延
	delay_path[clusterNum] = delay_path[max_idx] + 0.000000005;
	delay_path[clusterNum + 1] = delay_path[max_idx] + 0.00000001;
	delay_path[clusterNum + 2] = delay_path[max_idx_2nd] + 0.000000005;
	delay_path[clusterNum + 3] = delay_path[max_idx_2nd] + 0.00000001;

	//强功率簇分簇后调整簇的功率大小
	double power_max_idx = power_path[max_idx];                   //保存原来的值
	double power_max_idx_2nd = power_path[max_idx_2nd];     //保存原来的值

	power_path[max_idx] = power_max_idx * 10. / 20.;//原簇，即分割后的第一簇
	power_path[max_idx_2nd] = power_max_idx_2nd * 10. / 20.;

	power_path[clusterNum] = power_max_idx  * 6. / 20.;//分割后的第二簇
	power_path[clusterNum + 1] = power_max_idx * 4. / 20.;

	power_path[clusterNum + 2] = power_max_idx_2nd  * 6. / 20.;
	power_path[clusterNum + 3] = power_max_idx_2nd  * 4. / 20.;

	//强功率簇分簇后调整AOA、AOD
	arrival_angles_ini[clusterNum] = arrival_angles_ini[max_idx];
	arrival_angles_ini[clusterNum + 1] = arrival_angles_ini[max_idx];
	arrival_angles_ini[clusterNum + 2] = arrival_angles_ini[max_idx_2nd];
	arrival_angles_ini[clusterNum + 3] = arrival_angles_ini[max_idx_2nd];

	depart_angles_ini[clusterNum] = depart_angles_ini[max_idx];
	depart_angles_ini[clusterNum + 1] = depart_angles_ini[max_idx];
	depart_angles_ini[clusterNum + 2] = depart_angles_ini[max_idx_2nd];
	depart_angles_ini[clusterNum + 3] = depart_angles_ini[max_idx_2nd];

	//设置分簇后的射线偏移角度
	for (int cluster_idx = 0; cluster_idx < clusterNum + 4; cluster_idx++)
	{
		if (cluster_idx == max_idx || cluster_idx == max_idx_2nd)
		{
			num_rays[cluster_idx] = 10;//每个簇的子射线数量
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
	num_paths += 4;//意义和clusternum意义一样

	//保存各射线簇的时延、功率，每条射线的到达角和发射角（限制范围）
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

	//AOA与AOD随机配对，通过AOA随机排列实现：产生相同数量的随机数，对每个随机数，获取比其小的随机数个数作为AOA重新排列的索引
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
//函数名  ：generate_XPR()
//函数功能：生成交叉极化比（考虑极化情况下）
//输入参数：无
//输出参数：无（对全局变量的修改）
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


//产生服从标准正态分布的随机数
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


