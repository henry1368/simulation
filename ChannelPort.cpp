#include<cassert>
#include "ChannelPort.h"
#include "string.h"
#include "parameter.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "position.h"
#include "AP.h"
#include "NetworkManager.h"
#include "AntennaArray.h"
#include"FFTMath.h"
#include"Mobility.h"

#include "ue-lte-phy.h"
#include "lte-Phy.h"
#include "ue-wifi-phy.h"                 
#include "wifi-Phy.h"
#include "lte-bandwidth-manager.h"
#include "wifi-bandwidth-manager.h"

const double light_speed = 3e8;		//光速
double RandUni();
double RandNorm();
double wrapAngleDistribution(double theta);
double delay_clusters[20 + 4] = {0.0};//初始延时径
double power_path_ini[20 + 4] = { 0.0 };//初始功率
/*******************大基站************************/
double initial_phase_vv_LOS = 0.0 ;
double initial_phase_hh_LOS =  0.0 ;

double initial_phase_vv[20][24] = { 0.0 };
double initial_phase_vh[20][24] = { 0.0 };
double initial_phase_hv[20][24] = { 0.0 };
double initial_phase_hh[20][24] = { 0.0 };
int         num_rays[24] = { 0 };              //存储每簇的射线数量（强功率簇簇分簇后）
double arrival_angles[20][24] = { 0.0 }; //存储最终的到达角
double depart_angles[20][24] = { 0.0 };  //存储最终的发射角
double XPR[20][24] = { 0.0 };            //存储交叉极化比（考虑极化情况下）
int num_paths = 0;//一个扇区对应一条链路，存储每条链路的路径数量


ChannelPort::ChannelPort(){};
ChannelPort::ChannelPort(char *type)
{

	if (strcmp(type, "lte") == 0)
	{
        //一些常用参数的赋值

		Fre_interval = 0.015;                                      //子载波间隔15khz
		numsubcarrier = int(RBs_FOR_LTE * 12);                     //每个RB由12个子载波构成
		fftSize = int(pow(2, ceil(log(numsubcarrier) / log(double(2)))));//2048 点FFT 指数和对数函数=2^【不小于（log2(numsubcarrier)）】; 
		sample_rate = long(fftSize* Fre_interval * 1000000);//61.44M	
		sample_time = 1. / sample_rate;//单位s 	
	}

	else if (strcmp(type, "wifi") == 0)
	{
		Fre_interval = 0.3125;       //这个
		numsubcarrier = int(RBs_FOR_WIFI);      //子载波数=总带宽/子载波带宽
		fftSize = 2 * int(pow(2, ceil(log(numsubcarrier) / log(double(2)))));		//1024点FFT点数
		sample_rate = 2 * long(fftSize* Fre_interval * 1000000);//160M	
		sample_time = 1. / sample_rate;//单位s 	
		
	}
}
ChannelPort::~ChannelPort(){};


void ChannelPort::Compute_eNodeBpathloss()             //计算大小基站的路损
{
	double d = 0.0;
	double pLOS = 0.0;
	int c = 300000000;
	switch (SCENARIO)
	{
	case 1:{

			   if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)     //室外场景
			   {
				   if (strcmp(SCENARIO_type, "UMi") == 0)      //城市微型蜂窝模型 MUi    
				   {
					   const double BS_HEIGHT = 10.0;   //基站高度
					   const double MS_HEIGHT = 1.5;    //用户高度
					   fc = 2.5;                        //中心频率
					   E_medi.fc_m = fc;
					   const double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;  //break point distance
					   for (int i = 0; i < nb_cell; i++)
					   {
						   d = sqrt(ue->Dis_UE_Enb_table[nb_sector * i] * ue->Dis_UE_Enb_table[nb_sector * i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));
					
						   pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
					       eNodeB_isLOS[i] = (RandUni() < pLOS);                                //判定到每个大基站视距

						   if (eNodeB_isLOS[i])   //LOS         //计算pathloss
						   {
							   if (d >= 10 && d < dBP)
								   eNodeB_path_loss[i] = 22 * log10(d) + 28 + 20 * log10(fc);
							   else if (d >= dBP && d < 5000)
								   eNodeB_path_loss[i] = 40 * log10(d) + 7.8 - 18 * log10(BS_HEIGHT - 1) - 18 * log10(MS_HEIGHT - 1) + 2 * log10(fc);
							   else
								   assert(false);      //距离d超出合理范围
						   }
						   else         //NLOS
						   {
							   if (d >= 10 && d < 2000)
								   eNodeB_path_loss[i] = 36.7 * log10(d) + 22.7 + 26 * log10(fc);
							   else
								   assert(false);       //距离d超出合理范围
						   }
					   }
				   }


				   if (strcmp(SCENARIO_type, "UMa") == 0)	  //城市宏蜂窝模型 UMa
				   {
					   const double BS_HEIGHT = 25.0;
					   const double MS_HEIGHT = 1.5;
					   const double W = 20.0;    //street width
					   const double h = 20.0;   //average builing height
					   fc = 2.0;
					   E_medi.fc_m = fc;
					   const double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c; //break point distance


					   for (int i = 0; i < nb_cell; i++)
					   {
						   d = sqrt(ue->Dis_UE_Enb_table[nb_sector * i] * ue->Dis_UE_Enb_table[nb_sector * i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));
						  
						   pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 63.0)) + exp(-d / 63.0);
						   eNodeB_isLOS[i] = (RandUni() < pLOS);                             //判定到每个大基站的视距

						   if (eNodeB_isLOS[i])   //LOS         //计算pathloss
						   {
							   if (d >= 10 && d < dBP)
								   eNodeB_path_loss[i] = 22.0 * log10(d) + 28.0 + 20.0 * log10(fc);
							   else if (d >= dBP && d < 5000)
								   eNodeB_path_loss[i] = 40.0 * log10(d) + 7.8 - 18.0 * log10(BS_HEIGHT - 1) - 18.0 * log10(MS_HEIGHT - 1) + 2.0 * log10(fc);
							   else
								   assert(false); //距离d超出合理范围
							   /*PL_dB[i] = 103.4 + 24.2*log10(d / 1000);*/
						   }
						   else         //NLOS
						   {
							   if (d >= 10 && d < 5000)
								   eNodeB_path_loss[i] = 161.04 - 7.1 * log10(W) + 7.5 * log10(h) - (24.37 - 3.7 * (h / BS_HEIGHT) * (h / BS_HEIGHT)) * log10(BS_HEIGHT)
								   + (43.42 - 3.1 * log10(MS_HEIGHT)) * (log10(d) - 3) + 20 * log10(fc) - (3.2 * log10(11.75*MS_HEIGHT) * log10(11.75*MS_HEIGHT) - 4.97);
							   else
								   assert(false); //距离d超出合理范围
							   /*PL_dB[i] = 131.1 + 42.8*log10(d / 1000);*/
						   }
					   }
				   }

				   if (strcmp(SCENARIO_type, "RMa") == 0)	 //农村宏区模型 RMa
				   {
					   const double BS_HEIGHT = 35.0;
					   const double MS_HEIGHT = 1.5;
					   const int W = 20;    //street width
					   const int h = 5;     //average builing height
					   fc = 0.8;
					   E_medi.fc_m = fc;
					   const double dBP = 2 * PI * BS_HEIGHT * MS_HEIGHT * fc * 1e9 / c; //break point distance

					   for (int i = 0; i < nb_cell; i++)
					   {
						   d = sqrt(ue->Dis_UE_Enb_table[nb_sector * i] * ue->Dis_UE_Enb_table[nb_sector* i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));
						   pLOS = ((d < 10) ? 1 : exp(-(d - 10) / 1000));
						   eNodeB_isLOS[i] = (RandUni() < pLOS);                //判定到每个大基站的视距

						   if (eNodeB_isLOS[i]) //LOS
						   {
							   if (d >= 10 && d < dBP)
								   eNodeB_path_loss[i] = 20 * log10(40 * PI *d * fc / 3) + min(0.03*pow(h, 1.72), 10.0) * log10(d)
								   - min(0.044*pow(h, 1.72), 14.77) + 0.002 * log10(h)*d;
							   else if (d >= dBP && d < 10000)
								   eNodeB_path_loss[i] = 20 * log10(40 * PI *dBP * fc / 3) + min(0.03*pow(h, 1.72), 10.0) * log10(dBP)
								   - min(0.044*pow(h, 1.72), 14.77) + 0.002 * log10(h)*dBP + 40 * log10(d / dBP);
							   else
								   assert(false);                          //距离d超出合理范围
						   }

						   else //NLOS
						   {
							   if (d > 10 && d < 5000)
								   eNodeB_path_loss[i] = 161.04 - 7.1 * log10(W) + 7.5 * log10(h) - (24.37 - 3.7 * (h / BS_HEIGHT) * (h / BS_HEIGHT)) * log10(BS_HEIGHT)
								   + (43.42 - 3.1 * log10(BS_HEIGHT)) * (log10(d) - 3) + 20 * log10(fc) - (3.2 * log10(11.75*MS_HEIGHT) * log10(11.75*MS_HEIGHT) - 4.97);
							   else
								   assert(false); //距离d超出合理范围
						   }
					   }
				   }

				   if (strcmp(SCENARIO_type, "SMa") == 0)	   // 郊区宏小区模型SMa
				   {
					   const double BS_HEIGHT = 35.0;
					   const double MS_HEIGHT = 1.5;
					   const int W = 20; //street width
					   const int h = 10; //average builing height
					   fc = 2.5;
					   E_medi.fc_m = fc;
					   const double dBP = 2 * PI * BS_HEIGHT * MS_HEIGHT * fc * 1e9 / c; //break point distance
					   for (int i = 0; i<nb_cell; i++)
					   {
						   d = sqrt(ue->Dis_UE_Enb_table[nb_sector * i] * ue->Dis_UE_Enb_table[nb_sector * i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));
						   pLOS = ((d < 10) ? 1 : exp(-(d - 10) / 200));
						   eNodeB_isLOS[i] = (RandUni() < pLOS);                //判定到每个大基站的视距

						   if (eNodeB_isLOS[i]) //LOS
						   {
							   if (d > 10 && d < dBP)
								   eNodeB_path_loss[i] = 20 * log10(40 * PI *d * fc / 3) + min(0.03*pow(h, 1.72), 10.0) * log10(d)
								   - min(0.044*pow(h, 1.72), 14.77) + 0.002 * log10(h)*d;
							   else if (d > dBP && d < 5000)
								   eNodeB_path_loss[i] = 20 * log10(40 * PI *dBP * fc / 3) + min(0.03*pow(h, 1.72), 10.0) * log10(dBP)
								   - min(0.044*pow(h, 1.72), 14.77) + 0.002 * log10(h)*dBP + 40 * log10(d / dBP);
							   else
								   assert(false);                          //距离d超出合理范围
						   }

						   else //NLOS
						   {
							   if (d > 10 && d < 5000)
								   eNodeB_path_loss[i] = 161.04 - 7.1 * log10(W) + 7.5 * log10(h)
								   - (24.37 - 3.7 * (h / BS_HEIGHT) * (h / BS_HEIGHT)) * log10(BS_HEIGHT)
								   + (43.42 - 3.1 * log10(BS_HEIGHT)) * (log10(d) - 3) + 20 * log10(fc)
								   - (3.2 * log10(11.75*MS_HEIGHT) * log10(11.75*MS_HEIGHT) - 4.97);
							   else
								   assert(false); //距离d超出合理范围
						   }
					   }
				   }

				   //计算穿透损耗  
				   double O2VshadowFading = 5.*RandNorm();

				   if ((strcmp(SCENARIO_type, "UMa") == 0) || (strcmp(SCENARIO_type, "RMa") == 0))
				   {
					   for (int i = 0; i < nb_cell; i++)
						   eNodeB_path_loss[i] += 9 + O2VshadowFading;       //宏基站	
				   }

				   if (strcmp(SCENARIO_type, "SMa") == 0)       //用户处于室内或车里，穿透损耗不同
				   {
					   bool isIndoor = ue->GetMobilityModel()->m_isIndoor;
					   if (isIndoor)
					   {
						   for (int i = 0; i < nb_cell; i++)
							   eNodeB_path_loss[i] += 20.;                //穿墙损耗20dB		

					   }
					   else
					   {
						   for (int i = 0; i < nb_cell; i++)
							   eNodeB_path_loss[i] = 9. + O2VshadowFading;

					   }
				   }


				   if (strcmp(SCENARIO_type, "UMi") == 0)
				   {
					   bool isIndoor = ue->GetMobilityModel()->m_isIndoor;
					   if (isIndoor)
					   {
						   double tempDistance = 0.0;
						   double indoorLoss = 0.0;
						   for (int i = 0; i < nb_cell; i++)
						   {
							   indoorLoss = 0.5 * 25 * RandUni();
							   eNodeB_path_loss[i] += indoorLoss + 20;
						   }
					   }
				   }

				   //馈线损耗
				   for (int i = 0; i < nb_cell; i++)
					   eNodeB_path_loss[i] += 2;

				   //计算LSPs和阴影衰落

				   for (int i = 0; i < nb_cell; i++)
				   {
					   generateLSPs(SCENARIO_type, eNodeB_isLOS[i], eNodeB_LSPs[i], NULL);               //进行空间滤波时修改NULL为eNodeB_Ksi[i]
					   eNodeB_shadow_loss[i] = 10.*log10(eNodeB_LSPs[i][3]);                       //转换成dB
				   }


				   //计算天线增益
				   /*
				   方位角（azimuth），相对于物理的北而言，方位角在地面以下的一个角度。按照顺时针角度从北开始，
				   0度方位角表示正北，90度方位角表示正东，180度方位角表示正南，270度方位角表示正西，360度方位角表示角度回归，依然是正北。
				   仰角（elevation），也叫做海拔高度。当方位角测量完毕之后，需要用仰角来描述被观察物体相对于观察者的高度,如果观察者在地面上，那么仰角范围就在0度到90度之间
				   */
				   double eNodeB_antennaGain[nb_cell*nb_sector] = { 0. };
				   AntennaArray* antennaBs = new AntennaArray(SCENARIO_type, 0.); //生成天线类用于计算增益
				   NetworkManager* nm = NetworkManager::Init();
				   std::vector<ENodeB*> * m_ENodeBContainer = nm->GetENodeBContainer();
				   Position* Ue_pos = ue->Getposition();
				   double UE_X = Ue_pos->GetPositionX();  //用户的横、纵坐标
				   double UE_Y = Ue_pos->GetPositionY();
				   for (int i = 0; i < nb_cell; i++)
				   {
					   Position* Bs_pos = m_ENodeBContainer->at(nb_sector * i)->Getposition();
					   elevation[i] = atan((antennaBs->Bs_height - antennaBs->Ut_height) / ue->Dis_UE_Enb_table[nb_sector * i])*180. / PI;	//求出仰角 [-pi/2 , pi/2]	
					   double theta = atan2(UE_Y - Bs_pos->GetPositionY(), UE_X - Bs_pos->GetPositionX())*180. / PI;  //与X正轴夹角（-180，180）
					   thetaBS[i] = wrapAngleDistribution(-theta + 90);              //以正北方向为正方向的位置角度
					   thetaMS[i] = wrapAngleDistribution(-theta + 90 + 180.);

					   if (theta < 0)
						   theta = 360 + theta;                                   //逆时针到x正轴方向的夹角

					   for (int j = 0; j < nb_sector; j++)          //计算天线增益时求的是角度之比的平方，绝对值也可以
					   {
						   double thetaBS = wrapAngleDistribution(30 + 120 * j - theta);                 //求出每个扇区的方位角
						   //		cout << "用户"<<ue->GetIDNetworkNode()<<"与第" << i * nb_sector + j << "个扇区的方位角为" << thetaBS << endl;
						   eNodeB_antennaGain[i * nb_sector + j] = antennaBs->GetGain(thetaBS, elevation[i]);
					   }
				   }


				   //计算总能量损耗

				   for (int i = 0; i < nb_cell; i++)
				   {
					   eNodeB_power_loss[nb_sector*i] = -eNodeB_path_loss[i] - eNodeB_shadow_loss[i] + eNodeB_antennaGain[nb_sector* i];
					   eNodeB_power_loss[nb_sector * i + 1] = -eNodeB_path_loss[i] - eNodeB_shadow_loss[i] + eNodeB_antennaGain[nb_sector * i + 1];
					   eNodeB_power_loss[nb_sector * i + 2] = -eNodeB_path_loss[i] - eNodeB_shadow_loss[i] + eNodeB_antennaGain[nb_sector * i + 2];   //所有扇区
				   }
				   delete antennaBs;
			   }
			   break;
	}
	case 2:{}
	case 3:{}
	}
}
	
void ChannelPort::Compute_HeNodeBpathloss()             //计算小基站的路损
{
	char *ch;   //场景

   if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)
	{
	    ch = "UMi";                                //室外场景小基站统一按照UMi信道计算，室内按照TC1进行
	    double d = 0.0;
	    double pLOS = 0.0;
	    int c = 300000000;
       	const double BS_HEIGHT = 10.0;   //基站高度
		const double MS_HEIGHT = 1.5;    //用户高度
		fc = 3.5;                         //中心频率
		HE_medi.fc_m = fc;
		const double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;  //break point distance	
		for (int i = 0; i < nb_totalHenb; i++)
		{
			d = sqrt(ue->Dis_UE_Henb_table[i] * ue->Dis_UE_Henb_table[i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT)); 
			pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
			HeNodeB_isLOS[i] = (RandUni() < pLOS);                                  //判定到每个小基站的视距

			if (HeNodeB_isLOS[i])   //LOS         //计算pathloss
			{
				if (d >= 0 && d < dBP)     //把10改为0，防止小基站时发生assert(false)
					HeNodeB_path_loss[i] = 22 * log10(d) + 28 + 20 * log10(fc);
				else if (d >= dBP && d < 5000)
					HeNodeB_path_loss[i] = 40 * log10(d) + 7.8 - 18 * log10(BS_HEIGHT - 1) - 18 * log10(MS_HEIGHT - 1) + 2 * log10(fc);
				else
					assert(false);      //距离d超出合理范围
			}
			else         //NLOS
			{
				if (d >= 0 && d < 2000)
					HeNodeB_path_loss[i] = 36.7 * log10(d) + 22.7 + 26 * log10(fc);
				else
					assert(false);       //距离d超出合理范围
			}
		}
	}

	if (strcmp(SCENARIO_type, "TC1") == 0)        //目前按照WINNER II A1信道模型 计算  P44 D1.1.2 V1.2
	{
		ch = "TC1";
		double d = 0.0;
		double pLOS = 0.0;
		int c = 300000000;
		const double BS_HEIGHT = 2.9;     //基站高度
		const double MS_HEIGHT = 1.5;     //用户高度
		fc = 3.5;                         //中心频率
		HE_medi.fc_m = fc;	
		for (int i = 0; i < nb_totalHenb; i++)
		{		
			d = sqrt(ue->Dis_UE_Henb_table[i] * ue->Dis_UE_Henb_table[i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));             
			if (d <= 2.5)
				pLOS = 1;
			else
				pLOS = 1 - 0.9*pow((1 - pow((1.24 - 0.61*log10(d)), 3)), 0.3333);

			HeNodeB_isLOS[i] = (RandUni() < pLOS);                         //判定到每个小基站的视距


			if (HeNodeB_isLOS[i])        //LOS         //计算pathloss
			{				
				if (d >= 0 && d < 100)             //把3改为0，防止小基站时发生assert(false)
				 HeNodeB_path_loss[i] = 18.7 * log10(d) + 46.8 + 20 * log10(fc/5);
				else
					assert(false);                //距离d超出合理范围
			}
			else         //NLOS
			{
				if (d >= 0 && d < 100)
					HeNodeB_path_loss[i] = 36.8 * log10(d) + 43.8 + 26 * log10(fc/5);
				else
					assert(false);       //距离d超出合理范围
			}
		}
	}


	//计算穿透损耗
		bool isIndoor = ue->GetMobilityModel()->m_isIndoor;
		if (isIndoor)
		{
			double tempDistance = 0.0;
			double indoorLoss = 0.0;
			
			for (int i = 0; i < nb_totalHenb; i++)
			{
				indoorLoss = 0.5 * 25 * RandUni();
				HeNodeB_path_loss[i] += indoorLoss + 20;
			}
		}

	//馈线损耗
	
	for (int i = 0; i < nb_totalHenb; i++)
		HeNodeB_path_loss[i] += 2;

	//计算LSPs和阴影衰落	

	for (int i = 0; i < nb_totalHenb; i++)
	{
		generateLSPs(ch, HeNodeB_isLOS[i], HeNodeB_LSPs[i], NULL);     //进行空间滤波时修改NULL为HeNodeB_Ksi[i]
		HeNodeB_shadow_loss[i] = 10.*log10(HeNodeB_LSPs[i][3]);
	}

	//计算天线增益
	/*
	方位角（azimuth），相对于物理的北而言，方位角在地面以下的一个角度。按照顺时针角度从北开始，
	0度方位角表示正北，90度方位角表示正东，180度方位角表示正南，270度方位角表示正西，360度方位角表示角度回归，依然是正北。
	仰角（elevation），也叫做海拔高度。当方位角测量完毕之后，需要用仰角来描述被观察物体相对于观察者的高度,如果观察者在地面上，那么仰角范围就在0度到90度之间
	*/
	
	double HeNodeB_antennaGain[nb_totalHenb] = { 0. };   //初始化天线增益
	AntennaArray* antennaHeBs = new AntennaArray(ch,0.); //生成天线类用于计算增益
	NetworkManager* nm = NetworkManager::Init();	
	std::vector<HeNodeB*>* m_HeNodeBContainer = nm->GetHeNodeBContainer();
	Position* Ue_pos = ue->Getposition();
	double UE_X = Ue_pos->GetPositionX();  //用户的横、纵坐标
	double UE_Y = Ue_pos->GetPositionY();	
	for (int i = 0; i < nb_totalHenb; i++)
	{
		Position* HBs_pos = m_HeNodeBContainer->at(i)->Getposition();
		//求出俯仰角
		Helevation[i] = atan((antennaHeBs->Bs_height - antennaHeBs->Ut_height) / ue->Dis_UE_Henb_table[i])*180. / PI;
		//水平方向的角度
		double theta = atan2(UE_Y - HBs_pos->GetPositionY(), UE_X - HBs_pos->GetPositionX())*180. / PI;

		HethetaBS[i] = wrapAngleDistribution(-theta+90);
		HethetaMS[i] = wrapAngleDistribution(-theta +90+ 180.);

		if (theta < 0)
			theta = 360 + theta;              //逆时针到x正轴方向的夹角
			
		//小基站为全向天线，因此天线水平增益为0dB，垂直增益与俯仰角有关
		HeNodeB_antennaGain[i] = antennaHeBs->GetHeGain( Helevation[i]);	
	}	
		
	for (int i = 0; i <nb_totalHenb; i++)
		HeNodeB_power_loss[i] = -HeNodeB_path_loss[i] - HeNodeB_shadow_loss[i] + HeNodeB_antennaGain[i];
	delete antennaHeBs;
}


void ChannelPort::Compute_Wifipathloss()             //计算AP的路损
{

	double d = 0.0;
	double pLOS = 0.0;
	int c = 300000000;

    //城市微型蜂窝模型 MUi
	{
		const double BS_HEIGHT = 10.0;   //基站高度
		const double MS_HEIGHT = 1.5;    //用户高度
		fc = 5.0;                 //中心频率
		AP_medi.fc_m = fc;
		const double dBP = 4 * (BS_HEIGHT - 1) * (MS_HEIGHT - 1) * fc * 1e9 / c;  //break point distance	

		for (int i = 0; i < nb_totalAP; i++)
		{
			d = sqrt(ue->Dis_UE_AP_table[i] * ue->Dis_UE_AP_table[i] + (BS_HEIGHT - MS_HEIGHT) * (BS_HEIGHT - MS_HEIGHT));                                            //大基站和小基站的高度？？？？？？？？？？？？？？？？
			pLOS = min(18.0 / d, 1.0) * (1 - exp(-d / 36.0)) + exp(-d / 36.0);
			AP_isLOS[i] = (RandUni() < pLOS);                //判定到每个小基站的视距
			if (AP_isLOS[i])   //LOS         //计算pathloss
			{
				if (d >0 && d < dBP)
					AP_path_loss[i] = 22 * log10(d) + 28 + 20 * log10(fc);
				else if (d >= dBP && d < 5000)
					AP_path_loss[i] = 40 * log10(d) + 7.8 - 18 * log10(BS_HEIGHT - 1) - 18 * log10(MS_HEIGHT - 1) + 2 * log10(fc);
				else
					assert(false);      //距离d超出合理范围
			}
			else         //NLOS
			{
				if (d >0 && d < 2000)
					AP_path_loss[i] = 36.7 * log10(d) + 22.7 + 26 * log10(fc);
				else
					assert(false);       //距离d超出合理范围
			}
		}
	}

	//计算穿透损耗
	
	{		
		bool isIndoor = ue->GetMobilityModel()->m_isIndoor;
		if (isIndoor)
		{
			double tempDistance = 0.0;
			double indoorLoss = 0.0;

			for (int i = 0; i < nb_totalAP; i++)
			{
				indoorLoss = 0.5 * 25 * RandUni();
				AP_path_loss[i] += indoorLoss + 20;
			}
		}
	}

	//馈线损耗

	for (int i = 0; i < nb_totalAP; i++)
		AP_path_loss[i] += 2;

	//计算LSPs和阴影衰落	

	for (int i = 0; i < nb_totalAP; i++)
	{
		double *ptr = NULL;
		generateLSPs("UMi", AP_isLOS[i], AP_LSPs[i],ptr);                //WIFI信道暂时不进行空间滤波
		AP_shadow_loss[i] = 10.*log10(AP_LSPs[i][3]);
	}

	//计算天线增益
	/*
	方位角（azimuth），相对于物理的北而言，方位角在地面以下的一个角度。按照顺时针角度从北开始，
	0度方位角表示正北，90度方位角表示正东，180度方位角表示正南，270度方位角表示正西，360度方位角表示角度回归，依然是正北。
	仰角（elevation），也叫做海拔高度。当方位角测量完毕之后，需要用仰角来描述被观察物体相对于观察者的高度,如果观察者在地面上，那么仰角范围就在0度到90度之间
	*/

	double Wifi_antennaGain[nb_totalAP] = { 0. };   //初始化天线增益
	AntennaArray* antennaAP = new AntennaArray("UMi",0.); //生成天线类用于计算增益
	NetworkManager* nm = NetworkManager::Init();
	std::vector<AP*>*m_APContainer = nm->GetAPContainer();
	Position* Ue_pos = ue->Getposition();
	double UE_X = Ue_pos->GetPositionX();  //用户的横、纵坐标
	double UE_Y = Ue_pos->GetPositionY();
	for (int i = 0; i < nb_totalAP; i++)
	{
		Position* AP_pos = m_APContainer->at(i)->Getposition();
		//求出俯仰角
		APelevation[i] = atan((10. - antennaAP->Ut_height) / ue->Dis_UE_AP_table[i])*180. / PI;
		//水平方向的角度
		double theta = atan2(UE_Y - AP_pos->GetPositionY(), UE_X - AP_pos->GetPositionX())*180. / PI;

		APthetaBS[i] = wrapAngleDistribution(-theta+90);
		APthetaMS[i] = wrapAngleDistribution(-theta+90 + 180.);

		if (theta < 0)
			theta = 360 + theta;              //逆时针到x正轴方向的夹角
				//Wifi热点为全向天线，因此天线水平增益为0dB，垂直增益与俯仰角有关
		Wifi_antennaGain[i] = antennaAP->GetHeGain(APelevation[i]);
	}
	for (int i = 0; i <nb_totalAP; i++)
		AP_power_loss[i] = -AP_path_loss[i] - AP_shadow_loss[i] + Wifi_antennaGain[i];
	delete antennaAP;
}




void ChannelPort::generateLSPs(char* str,bool LOSflag, double *lsp,double *ksi)
{
	double DS_mu, DS_sigma, ASD_mu, ASD_sigma, ASA_mu, ASA_sigma, SF_sigma, K_mu, K_sigma;
	double sqrt_corr_matrix[5][5] = {0.0};

	if (strcmp(str, "UMi") == 0 || strcmp(str, "TC1") == 0)
	{
		if (LOSflag == 1)//城市微小区的可视场景
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

		else//城市微小区的非可视场景
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
	}
	if (strcmp(str, "UMa") == 0)
	{
		if (LOSflag == 1)//城市宏小区的可视场景
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
		else//城市宏小区的非可视场景
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
	}
	if (strcmp(str, "RMa") == 0)
	{
		if (LOSflag == 1)//乡村宏小区的可视场景
		{
			sqrt_corr_matrix[0][0] = 0.8540; sqrt_corr_matrix[0][1] = -0.0407; sqrt_corr_matrix[0][2] = 0.4238; sqrt_corr_matrix[0][3] = -0.2989; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = -0.0407; sqrt_corr_matrix[1][1] = 0.9588; sqrt_corr_matrix[1][2] = -0.0234; sqrt_corr_matrix[1][3] = -0.2803; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.4238; sqrt_corr_matrix[2][1] = -0.0234; sqrt_corr_matrix[2][2] = 0.8795; sqrt_corr_matrix[2][3] = -0.2151; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.2989; sqrt_corr_matrix[3][1] = -0.2803; sqrt_corr_matrix[3][2] = -0.2151; sqrt_corr_matrix[3][3] = 0.8865; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -7.49;
			DS_sigma = 0.55;
			ASD_mu = 0.90;
			ASD_sigma = 0.38;
			ASA_mu = 1.52;
			ASA_sigma = 0.24;
			SF_sigma = 4;
			K_mu = 7;
			K_sigma = 4;
		}
		else//乡村宏小区的非可视场景
		{
			sqrt_corr_matrix[0][0] = 0.8889; sqrt_corr_matrix[0][1] = -0.0287; sqrt_corr_matrix[0][2] = 0.3941; sqrt_corr_matrix[0][3] = -0.2318; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = -0.0287; sqrt_corr_matrix[1][1] = 0.9769; sqrt_corr_matrix[1][2] = 0.0116; sqrt_corr_matrix[1][3] = -0.2116; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.3941; sqrt_corr_matrix[2][1] = 0.0116; sqrt_corr_matrix[2][2] = 0.9176; sqrt_corr_matrix[2][3] = 0.0503; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.2318; sqrt_corr_matrix[3][1] = -0.2116; sqrt_corr_matrix[3][2] = 0.0503; sqrt_corr_matrix[3][3] = 0.9481; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -7.43;
			DS_sigma = 0.48;
			ASD_mu = 0.95;
			ASD_sigma = 0.45;
			ASA_mu = 1.52;
			ASA_sigma = 0.13;
			SF_sigma = 8;
			K_mu = 0;
			K_sigma = 0;
		}
	}
	if (strcmp(str, "SMa") == 0)
	{
		if (LOSflag == 1)//郊区宏小区的可视场景
		{
			sqrt_corr_matrix[0][0] = 0.9659; sqrt_corr_matrix[0][1] = 0.0000; sqrt_corr_matrix[0][2] = 0.0000; sqrt_corr_matrix[0][3] = -0.2588; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = 0.0000; sqrt_corr_matrix[1][1] = 1.0000; sqrt_corr_matrix[1][2] = 0.0000; sqrt_corr_matrix[1][3] = 0.0000; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.0000; sqrt_corr_matrix[2][1] = 0.0000; sqrt_corr_matrix[2][2] = 1.0000; sqrt_corr_matrix[2][3] = 0.0000; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.2588; sqrt_corr_matrix[3][1] = 0.0000; sqrt_corr_matrix[3][2] = 0.0000; sqrt_corr_matrix[3][3] = 0.9659; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -7.23;
			DS_sigma = 0.38;
			ASD_mu = 0.78;
			ASD_sigma = 0.12;
			ASA_mu = 1.48;
			ASA_sigma = 0.20;
			SF_sigma = 4;
			K_mu = 9;
			K_sigma = 7;
		}
		else//郊区宏小区的非可视场景
		{
			sqrt_corr_matrix[0][0] = 0.9556; sqrt_corr_matrix[0][1] = -0.1735; sqrt_corr_matrix[0][2] = 0.0000; sqrt_corr_matrix[0][3] = -0.2384; sqrt_corr_matrix[0][4] = 0.0000;
			sqrt_corr_matrix[1][0] = -0.1735; sqrt_corr_matrix[1][1] = 0.9380; sqrt_corr_matrix[1][2] = 0.0000; sqrt_corr_matrix[1][3] = 0.3001; sqrt_corr_matrix[1][4] = 0.0000;
			sqrt_corr_matrix[2][0] = 0.0000; sqrt_corr_matrix[2][1] = 0.0000; sqrt_corr_matrix[2][2] = 1.0000; sqrt_corr_matrix[2][3] = 0.0000; sqrt_corr_matrix[2][4] = 0.0000;
			sqrt_corr_matrix[3][0] = -0.2384; sqrt_corr_matrix[3][1] = 0.3001; sqrt_corr_matrix[3][2] = 0.0000; sqrt_corr_matrix[3][3] = 0.9237; sqrt_corr_matrix[3][4] = 0.0000;
			sqrt_corr_matrix[4][0] = 0.0000; sqrt_corr_matrix[4][1] = 0.0000; sqrt_corr_matrix[4][2] = 0.0000; sqrt_corr_matrix[4][3] = 0.0000; sqrt_corr_matrix[4][4] = 1.0000;

			DS_mu = -7.12;
			DS_sigma = 0.33;
			ASD_mu = 0.90;
			ASD_sigma = 0.36;
			ASA_mu = 1.65;
			ASA_sigma = 0.25;
			SF_sigma = 8;
			K_mu = 0;
			K_sigma = 0;
		}
	}

	double cross_corr_delay = 0, cross_corr_aoa = 0, cross_corr_aod = 0, cross_corr_shadow = 0, cross_corr_kfactor = 0;


	double Gaussrand_Num[5] = { 0 };
	for (int idx = 0; idx < 5; idx++)
	{
	   
		if (ksi==NULL)
			Gaussrand_Num[idx] = RandNorm();
		else
		    Gaussrand_Num[idx] = ksi[idx];      //改随机生成为
	}

	for (int idx = 0; idx < 5; idx++)
	{
		cross_corr_delay = cross_corr_delay + sqrt_corr_matrix[0][idx] * Gaussrand_Num[idx];
		cross_corr_aod = cross_corr_aod + sqrt_corr_matrix[1][idx] * Gaussrand_Num[idx];
		cross_corr_aoa = cross_corr_aoa + sqrt_corr_matrix[2][idx] * Gaussrand_Num[idx];
		cross_corr_shadow = cross_corr_shadow + sqrt_corr_matrix[3][idx] * Gaussrand_Num[idx];
		cross_corr_kfactor = cross_corr_kfactor + sqrt_corr_matrix[4][idx] * Gaussrand_Num[idx];
	}
	lsp[0] = pow(10, DS_mu + DS_sigma * cross_corr_delay);//延时的单位为s
	lsp[1] = pow(10, ASD_mu + ASD_sigma * cross_corr_aod);//均为线性单位
	lsp[2] = pow(10, ASA_mu + ASA_sigma * cross_corr_aoa);
	lsp[3] = pow(10, 0.1*SF_sigma * cross_corr_shadow);
	lsp[4] = pow(10, 0.1*(K_mu + K_sigma * cross_corr_kfactor));

	if (lsp[1] > 104)
		lsp[1] = 104;
	if (lsp[2] >104)
		lsp[2] = 104;

}




/**********小尺度信道产生程序*********************************************/
//Wifi的AP初始参数设置
void ChannelPort::setAPPoint(char sid)
{
	Sectorid = sid;
	double pathlosstemp = -AP_path_loss[Sectorid];
	double shadowFadingtemp = -AP_shadow_loss[Sectorid];

	LOSflag = AP_isLOS[Sectorid];//到某个基站的视距标志

	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = APthetaBS[Sectorid];//在生成簇到达角时作为视距方向应用
	phi_AoA_LOS = APthetaMS[Sectorid];
	
	
}
//小基站初始参数设置
void ChannelPort::setHePoint(char sid)
{	
	Sectorid = sid;
	double pathlosstemp = -HeNodeB_path_loss[Sectorid];
	double shadowFadingtemp = -HeNodeB_shadow_loss[Sectorid];
	LOSflag = HeNodeB_isLOS[Sectorid];//到某个基站的视距标志
	
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = HethetaBS[Sectorid];//在生成簇到达角时作为视距方向应用
	phi_AoA_LOS = HethetaMS[Sectorid];

	
}
//大基站初始参数设置
void ChannelPort::setEPoint( char sid)
{
	Sectorid = sid;
	double pathlosstemp = -eNodeB_path_loss[Sectorid / nb_sector];
	double shadowFadingtemp = -eNodeB_shadow_loss[Sectorid / nb_sector];
	LOSflag = eNodeB_isLOS[Sectorid / nb_sector];//到某个基站的视距标志	
	pathLoss = pow(10., 0.1*pathlosstemp);
	shadowFading = pow(10., 0.1*shadowFadingtemp);
	phi_AoD_LOS = thetaBS[Sectorid / nb_sector];//在生成簇到达角时作为视距方向应用
	phi_AoA_LOS = thetaMS[Sectorid / nb_sector];
	
}

void ChannelPort::initial()
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
void ChannelPort::set_small_scale_parameter(char * ch)
{
	if (strcmp(ch, "UMi") == 0 || strcmp(ch, "TC1") == 0)
	{
		if (LOSflag == 1)         //城市微小区的可视传输
		{
			rtau = 3.2;             //时延缩放参数
			XPR_mu = 9;             //Cross-polarization ratio  交叉极化比
			XPR_sigma = 0;// 3;          //哪来的？？？？？
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
    if (strcmp(ch, "UMa") == 0)//UMa
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
	if (strcmp(ch, "SMa") == 0)//Sma
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
	if (strcmp(ch, "RMa") == 0)//Rma
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
void ChannelPort::initial_Array()//对后面计算需要的数组进行初始化。最后记得函数释放内存空间
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
//大基站扩展参数
void ChannelPort::generate_ELSPsigma()
{
	delay_spread = eNodeB_LSPs[Sectorid / nb_sector][0];
	AoD_spread = eNodeB_LSPs[Sectorid / nb_sector][1];
	AoA_spread = eNodeB_LSPs[Sectorid / nb_sector][2];
	K_factor = eNodeB_LSPs[Sectorid / nb_sector][4];
	K_factor_db = 10. * log10(K_factor);
}
//小基站扩展参数
void ChannelPort::generate_HeLSPsigma()
{
	delay_spread = HeNodeB_LSPs[Sectorid ][0];
	AoD_spread = HeNodeB_LSPs[Sectorid ][1];
	AoA_spread = HeNodeB_LSPs[Sectorid ][2];
	K_factor = HeNodeB_LSPs[Sectorid ][4];
	K_factor_db = 10. * log10(K_factor);
}

//////////////////////////////////////////////////////////////////////////
//函数名   ：generate_delay()
//函数功能：产生信道中的延迟
//输入参数：产生当前用户到某一扇区所有簇的时延径
//输出参数：无(对全局变量的修改)
//////////////////////////////////////////////////////////////////////////
void ChannelPort::generate_delay()
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
		delay_clusters[i] = delay_clusters[i]- delaymin;
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
void ChannelPort::generate_Gauss()
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
void ChannelPort::generate_powers()
{

	double powersum = 0;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i] = exp(-delay_path[i] * (rtau - 1.) / (rtau*delay_spread))*pow(10., -gaussRand[i] * 3. / 10.);//zita在所有场景是3dB，a*zita代表零均值，标准差3dB的高斯随机变量
		if (power_path_ini[i] < 0)
		{
			cout << "簇功率值为负数！值为：" << power_path_ini[i]<< endl;
			system("pause");
		}
		powersum = powersum + power_path_ini[i];
	}

	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i]= power_path_ini[i]/ powersum;//除以总功率，功率归一化
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
		power_path_ini[i]= 0;
		delay_clusters[i]= 0;//这两句似乎没有什么用
	}
	clusterNum = num_paths;
	for (int i = 0; i < clusterNum; i++)
	{
		power_path_ini[i]= power_path[i];
		delay_clusters[i]= delay_path[i];
	}
	for (int i = 0; i < clusterNum; i++)
	{
		power_path[i] = power_path_ini[i];
	}

	//LOS条件下功率调整
	if (LOSflag == 1)
	{
		power_path[0] = K_factor / (K_factor + 1) + power_path[0] / (K_factor + 1);
		power_path_ini[0]= power_path[0];
		for (int i = 1; i < clusterNum; i++)
		{
			power_path[i] = power_path[i] / (K_factor + 1);
			if (power_path[i] < 0)
			{
				cout << "LOS下簇功率值为负数！值为：" << power_path[i] << endl;
				system("pause");
			}
			power_path_ini[i]= power_path[i];
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
			delay_clusters[i]= delay_clusters[i] / D_factor;
			if (!_finite(abs(delay_clusters[i])))
			{
				cout << "第" << i << "条多径出现问题" << endl;
				cout << "delay_path[i]=" << delay_clusters[i]<< endl;
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
void ChannelPort::generate_c()
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
void ChannelPort::generate_AoAs()
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
void ChannelPort::generate_AoDs()
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
void ChannelPort::set_SubClusters()
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
			num_rays[cluster_idx]= 6;
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
			num_rays[cluster_idx]= 20;
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
		delay_clusters[clusterIdx]= delay_path[clusterIdx];
		if (clusterIdx == 0)
			power_path_ini[clusterIdx] = power_path[0] + single_los_ray_power;
		else
			power_path_ini[clusterIdx]= power_path[clusterIdx];
		for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
		{
			arrival_angles[rayIdx][clusterIdx] = arrival_angles_ini[clusterIdx] + Caoa*offset_angle[rayIdx][clusterIdx];
			arrival_angles[rayIdx][clusterIdx] = wrapAngleDistribution(arrival_angles[rayIdx][clusterIdx]);
			depart_angles[rayIdx][clusterIdx]= depart_angles_ini[clusterIdx] + Caod*offset_angle[rayIdx][clusterIdx];
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
			arrival_angles[i][clusterIdx]= temp_angle[temp_order[i]];
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
	
	void ChannelPort::generate_initial_phase()
	{
		
		initial_phase_vv_LOS = (RandUni() * 2 - 1) * PI;
		initial_phase_hh_LOS= (RandUni() * 2 - 1) * PI;
		for (int clusterIdx = 0; clusterIdx < clusterNum; clusterIdx++)
		{
			for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
			{
				initial_phase_vv[rayIdx][clusterIdx]= (RandUni() * 2 - 1) * PI;
				initial_phase_vh[rayIdx][clusterIdx] = (RandUni() * 2 - 1) * PI;
				initial_phase_hv[rayIdx][clusterIdx]= (RandUni() * 2 - 1) * PI;
				initial_phase_hh[rayIdx][clusterIdx]= (RandUni() * 2 - 1) * PI;
			}
		}
		
	}
	//////////////////////////////////////////////////////////////////////////
	//函数名  ：generate_XPR()
	//函数功能：生成交叉极化比（考虑极化情况下）
	//输入参数：无
	//输出参数：无（对全局变量的修改）
	//////////////////////////////////////////////////////////////////////////
	void ChannelPort::generate_XPR()
	{
		for (int clusterIdx = 0; clusterIdx < num_paths; clusterIdx++)
		{
			for (int rayIdx = 0; rayIdx < num_rays[clusterIdx]; rayIdx++)
			{
				XPR[rayIdx][clusterIdx] = pow(10., (XPR_mu + XPR_sigma * RandNorm()) / 10.);
			}
		}
	}
	void  ChannelPort::calculateNodeB_medi()
	{
		eNodeB_medi.clear();
		for (int N = 0; N < nb_totalEnb; N++)
		{
				setEPoint(N);
				initial();//初始化信道相关参量
				set_small_scale_parameter(SCENARIO_type);
				initial_Array();
				generate_ELSPsigma();
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
			for (int i = 0; i < 20; i++)	{for (int j = 0; j < 24; j++) {E_medi.initial_phase_vv_m[i][j] = initial_phase_vv[i][j]; }}
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.arrival_angles_m[i][j] = arrival_angles[i][j]; } }	
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.depart_angles_m[i][j] = depart_angles[i][j]; } }
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_vh_m[i][j] = initial_phase_vh[i][j]; } }
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_hv_m[i][j] = initial_phase_hv[i][j]; } }
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.initial_phase_hh_m[i][j] = initial_phase_hh[i][j]; } }	
			for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ E_medi.XPR_m[i][j] = XPR[i][j]; } }

			eNodeB_medi.push_back(E_medi);
		}	

	}
	void  ChannelPort::calculateNodeBSmallFading(int mscene, int time)//产生当前用户到所有扇区的信道系数
	{
	
		/*
		2015/6/27修改，只计算最强干扰部分的信道，根据用户存储的 ID_Enb[nb_totalEnb]和int ID_Henb[nb_totalHenb]选出若干最强干扰计算
		*/
		switch (SCENARIO)
		{
		case 1:{
				   if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)
				   {
					   for (int k = 0; k < nb_totalEnb; k++)
					   {
						   for (int i = 0; i < RBs_FOR_LTE; i++)                                   //初始化
						   {
							   channel_Ecoefficients[k][i] = MatrixXcd::Zero(Nr, Nt);
						   }
					   }

					   for (int KK = 0; KK < Num_Enb_Select + 1; KK++)    //计算个数为连接基站和若干宏干扰基站
					   {
						   int N = ue->ID_Enb[KK];
						   TTItime = time;
						   K_factor = eNodeB_LSPs[N / nb_sector][4];
						   LOSflag = eNodeB_isLOS[N / nb_sector];//到某个基站的视距标志
						   phi_AoD_LOS = thetaBS[N / nb_sector];//在生成簇到达角时作为视距方向应用
						   phi_AoA_LOS = thetaMS[N / nb_sector];
						   travelSpeed = ue->GetMobilityModel()->m_speed*1000. / 3600.; //单位m/s
						   travelAngle = ue->GetMobilityModel()->m_speedDirection;

						   complex <double> channel_temp[24][Nr][Nt];            //[射线簇数][UE天线数][基站天线数]
						   double d_u = 0;
						   double d_s = 0;

						   LN_size = eNodeB_medi.at(N).num_paths_m;//每个扇区的时延径数	
						   fc = eNodeB_medi.at(N).fc_m;
						   double elevationtemp = elevation[N / nb_sector];
						   double Lambda = light_speed / (fc*1e9); //载波波长
						   int Secindex = N % nb_sector;
						   double broadsideAzimuthBs = 60. - 120.*Secindex;        //基站天线宽边方向,以逆时针方向为正（扇区标号按逆时针方向）
						   AntennaArray* antennaBs = new AntennaArray(Antenna_IMTA_bs, broadsideAzimuthBs, mscene); //生成基站天线类
						   AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, broadsideAzimuthBs, mscene); //生成用户天线类
						   for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
						   {
							   //取未经尺度压缩的簇功率
							   double K = K_factor;
							   double power_n = eNodeB_medi.at(N).power_path_ini_m[clusterIdx];
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
									   for (int rayIdx = 0; rayIdx < eNodeB_medi.at(N).num_rays_m[clusterIdx]; rayIdx++)
									   {
										   double arr_angle = eNodeB_medi.at(N).arrival_angles_m[rayIdx][clusterIdx];
										   double dep_angle = eNodeB_medi.at(N).depart_angles_m[rayIdx][clusterIdx];

										   double v = travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
										   complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
										   complex<double> F_tx_v = antennaBs->getVerticalFieldPattern(dep_angle, elevationtemp, s);
										   complex <double> exp_vv(0, eNodeB_medi.at(N).initial_phase_vv_m[rayIdx][clusterIdx]);
										   complex <double> j2pivt(0, 2.*PI*v*TTItime*0.001);
										   complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
										   complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
										   double xpr = eNodeB_medi.at(N).XPR_m[rayIdx][clusterIdx];
										   complex<double> F_tx_h = antennaBs->getHorizontalFieldPattern(dep_angle, elevationtemp, s);
										   complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
										   complex <double> exp_vh(0, eNodeB_medi.at(N).initial_phase_vh_m[rayIdx][clusterIdx]);
										   complex <double> exp_hv(0, eNodeB_medi.at(N).initial_phase_hv_m[rayIdx][clusterIdx]);
										   complex <double> exp_hh(0, eNodeB_medi.at(N).initial_phase_hh_m[rayIdx][clusterIdx]);
										   //矩阵运算
										   channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
											   (F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / xpr)) +
											   F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / xpr) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
									   }

									   //这里做个标志			channel_temp[clusterIdx][u][s] *= sqrt(power_n);
									   //	channel_temp[clusterIdx][u][s] *= sqrt(power_n / eNodeB_medi.at(N).num_rays_m[clusterIdx]);
									   if (LOSflag == 1)
									   {
										   channel_temp[clusterIdx][u][s] *= sqrt(1 / (K + 1));
										   if (clusterIdx == 0)
										   {
											   complex<double> F_rx_v_LOS = antennaMs->getominaVerticalFieldPattern(phi_AoA_LOS, elevationtemp, u);
											   complex<double> F_tx_v_LOS = antennaBs->getVerticalFieldPattern(phi_AoD_LOS, elevationtemp, s);
											   double v_LOS = travelSpeed*cos((phi_AoA_LOS - travelAngle)*PI / 180.) / Lambda;
											   complex <double> exp_vv_LOS(0, eNodeB_medi.at(N).initial_phase_vv_LOS_m);
											   complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*TTItime*0.001);
											   complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
											   complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
											   complex<double> F_tx_h_LOS = antennaBs->getHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
											   complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
											   complex <double> exp_hh_LOS(0, eNodeB_medi.at(N).initial_phase_hh_LOS_m);
											   channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
												   sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
										   }
									   }
								   }
							   }
						   }
						   double pathloss = eNodeB_medi.at(N).pathLoss_m;
						   double shadow = eNodeB_medi.at(N).shadowFading_m;
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
									   k = int(eNodeB_medi.at(N).delay_clusters_m[i] / sample_time + 0.5);      //四舍五入，原来是向下取整
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
							   channel_Ecoefficients[N][i] = B;
						   }


						   //释放内存空间
						   delete[]fft_in;
						   delete[]fft_out;
						   delete antennaBs;
						   delete antennaMs;
					   }


				   }
				   break;
		}

		case 2:{}
		case 3:{}

		}


		
}

	
	void  ChannelPort::calculateHeNodeB_medi()
	{
		char* ch;
		if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)
			ch = "UMi";                                      //室外场景统一用"UMi"
		if (strcmp(SCENARIO_type, "TC1") == 0)
			ch = "TC1";

		HeNodeB_medi.clear();
		for (int N = 0; N < nb_totalHenb; N++)
		{
			    setHePoint(N);//小基站的基本参数设置		
			
				initial();//初始化信道相关参量

				set_small_scale_parameter(ch);

				initial_Array();

				generate_HeLSPsigma();

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
			

			//ChannMedi E_medi = { 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			HE_medi.num_paths_m = num_paths;
			HE_medi.pathLoss_m = pathLoss;
			HE_medi.shadowFading_m = shadowFading;
			
			HE_medi.initial_phase_vv_LOS_m = initial_phase_vv_LOS;
			HE_medi.initial_phase_hh_LOS_m = initial_phase_hh_LOS;
			for (int i = 0; i < 24; i++){ HE_medi.num_rays_m[i] = num_rays[i]; }
			for (int i = 0; i < 24; i++){HE_medi.delay_clusters_m[i] = delay_clusters[i]; }
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
	
void  ChannelPort::calculateHeNodeBSmallFading(int time)//产生当前用户到所有小基站的信道系数
{

//2015/6/27/修改只算最强干扰部分信道

	for (int k = 0; k < nb_totalHenb; k++)
	{
		for (int i = 0; i < RBs_FOR_LTE; i++)                                   //初始化
		{
			channel_Hecoefficients[k][i] = MatrixXcd::Zero(Nr, Nt);
		}
	}



	for (int KK = 0; KK < Num_Henb_Select+1; KK++)    //计算个数为连接基站和若干小干扰基站
	{
		int N = ue->ID_Henb[KK];
		K_factor = HeNodeB_LSPs[N ][4];
		LOSflag = HeNodeB_isLOS[N];//到某个基站的视距标志
		phi_AoD_LOS =HethetaBS[N];//在生成簇到达角时作为视距方向应用
		phi_AoA_LOS = HethetaMS[N];
		TTItime = time;
		travelSpeed = ue->GetMobilityModel()->m_speed*1000. / 3600.; //单位m/s
		travelAngle = ue->GetMobilityModel()->m_speedDirection;
		
		complex <double> channel_temp[24][Nr][Nt];            //[射线簇数][UE天线数][基站天线数]
		double d_u = 0;
		double d_s = 0;
		fc = HeNodeB_medi.at(N).fc_m;
		LN_size = HeNodeB_medi.at(N). num_paths_m;//每个扇区的时延径数
		//用户天线宽边方向，与天线类型有关，由于此处使用全向天线，故设置为0		
		double elevationtemp = Helevation[N];
		double Lambda = light_speed / (fc*1e9); //载波波长
		AntennaArray* antennaHeBs = new AntennaArray(Antenna_IMTA_bs, 0.,1); //生成基站天线类
		AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, 0.,1); //生成用户天线类

		for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
		{
			//取未经尺度压缩的簇功率
			double K = K_factor;
			double power_n = HeNodeB_medi.at(N).power_path_ini_m[clusterIdx];
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
						double arr_angle = HeNodeB_medi.at(N).arrival_angles_m[rayIdx][clusterIdx];
						double dep_angle = HeNodeB_medi.at(N).depart_angles_m[rayIdx][clusterIdx];

						double v = travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
						complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
						complex<double> F_tx_v = antennaHeBs->getHeVerticalFieldPattern(dep_angle, elevationtemp, s);
						complex <double> exp_vv(0, HeNodeB_medi.at(N).initial_phase_vv_m[rayIdx][clusterIdx]);
						complex <double> j2pivt(0, 2.*PI*v*TTItime*0.001);
						complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
						complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
						double xpr = HeNodeB_medi.at(N).XPR_m[rayIdx][clusterIdx];
						complex<double> F_tx_h = antennaHeBs->getHeHorizontalFieldPattern(dep_angle, elevationtemp, s);
						complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
						complex <double> exp_vh(0, HeNodeB_medi.at(N).initial_phase_vh_m[rayIdx][clusterIdx]);
						complex <double> exp_hv(0, HeNodeB_medi.at(N).initial_phase_hv_m[rayIdx][clusterIdx]);
						complex <double> exp_hh(0, HeNodeB_medi.at(N).initial_phase_hh_m[rayIdx][clusterIdx]);
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
							complex <double> exp_vv_LOS(0, HeNodeB_medi.at(N).initial_phase_vv_LOS_m);
							complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*TTItime*0.001);
							complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
							complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
							complex<double> F_tx_h_LOS = antennaHeBs->getHeHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
							complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
							complex <double> exp_hh_LOS(0, HeNodeB_medi.at(N).initial_phase_hh_LOS_m);
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
						}
					}
				}
			}
		}
		double pathloss = HeNodeB_medi.at(N).pathLoss_m;
		double shadow = HeNodeB_medi.at(N).shadowFading_m;

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
					k = int(HeNodeB_medi.at(N).delay_clusters_m[i] / sample_time + 0.5);      //四舍五入，原来是向下取整
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
			channel_Hecoefficients[N][i] = B;
		}


		//释放内存空间
		delete[]fft_in;
		delete[]fft_out;
		delete antennaHeBs;
		delete antennaMs;
	}
}


	
void  ChannelPort::calculateAP_medi()
{
	Wifi_medi.clear();
	for (int N = 0; N < nb_totalAP; N++)
	{
		setAPPoint(N);//Wifi的AP群的基本参数设置	
		
			initial();//初始化信道相关参量

			set_small_scale_parameter("UMi");

			initial_Array();

			generate_APLSPsigma();

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


		
		AP_medi.num_paths_m = num_paths;
		AP_medi.pathLoss_m = pathLoss;
		AP_medi.shadowFading_m = shadowFading;
		
		AP_medi.initial_phase_vv_LOS_m = initial_phase_vv_LOS;
	    AP_medi.initial_phase_hh_LOS_m = initial_phase_hh_LOS;
		for (int i = 0; i < 24; i++){ AP_medi.num_rays_m[i] = num_rays[i]; }
		for (int i = 0; i < 24; i++){ AP_medi.delay_clusters_m[i] = delay_clusters[i]; }
		for (int i = 0; i < 24; i++){ AP_medi.power_path_ini_m[i] = power_path_ini[i]; }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++) { AP_medi.initial_phase_vv_m[i][j] = initial_phase_vv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.arrival_angles_m[i][j] = arrival_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.depart_angles_m[i][j] = depart_angles[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.initial_phase_vh_m[i][j] = initial_phase_vh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.initial_phase_hv_m[i][j] = initial_phase_hv[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.initial_phase_hh_m[i][j] = initial_phase_hh[i][j]; } }
		for (int i = 0; i < 20; i++)	{ for (int j = 0; j < 24; j++){ AP_medi.XPR_m[i][j] = XPR[i][j]; } }

		Wifi_medi.push_back(AP_medi);
	}

}

	//计算Wifi的AP群的小尺度信道系数
void  ChannelPort::calculateAPSmallFading( int time)               //产生当前用户到所有AP群的信道系数
	{

	for (int k = 0; k < nb_totalAP; k++)
	  {
		  for (int i = 0; i < RBs_FOR_LTE; i++)                                   //初始化
		 {
			channel_APcoefficients[k][i] = MatrixXcd::Zero(Nr, Nt);
		 }
	  }


	    for (int KK = 0; KK< nb_apingroup; KK++)
		{

			int N = ue->ID_AP[KK];
			K_factor = AP_LSPs[N][4];
			LOSflag = AP_isLOS[N];//到某个基站的视距标志
			phi_AoD_LOS = APthetaBS[N];//在生成簇到达角时作为视距方向应用
			phi_AoA_LOS = APthetaMS[N];
			travelSpeed = ue->GetMobilityModel()->m_speed*1000. / 3600.; //单位m/s
			travelAngle = ue->GetMobilityModel()->m_speedDirection;
			TTItime = time;
		
			complex <double> channel_temp[24][Nr][Nt];            //[射线簇数][UE天线数][基站天线数]
			double d_u = 0;
			double d_s = 0;
			LN_size = Wifi_medi.at(N).num_paths_m;//每个扇区的时延径数
			fc = Wifi_medi.at(N).fc_m;
			//用户天线宽边方向，与天线类型有关，由于此处使用全向天线，故设置为0		
			double elevationtemp = APelevation[N];
			double Lambda = light_speed / (fc*1e9); //载波波长
			AntennaArray* antennaAP = new AntennaArray(Antenna_IMTA_bs,0., 1); //生成基站天线类
			AntennaArray* antennaMs = new AntennaArray(Antenna_IMTA_ms, 0.,1); //生成用户天线类

			for (int clusterIdx = 0; clusterIdx < LN_size; clusterIdx++)
			{
				//取未经尺度压缩的簇功率
				double K = K_factor;
				double power_n = Wifi_medi.at(N).power_path_ini_m[clusterIdx];
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
						d_s = antennaAP->getDistanceLambda(s) * Lambda;//基站天线间距10Lambda					
						channel_temp[clusterIdx][u][s] = (0, 0);  //[射线簇数][UE天线数][基站天线数]
						for (int rayIdx = 0; rayIdx < Wifi_medi.at(N).num_rays_m[clusterIdx]; rayIdx++)
						{
							double arr_angle = Wifi_medi.at(N).arrival_angles_m[rayIdx][clusterIdx];
							double dep_angle = Wifi_medi.at(N).depart_angles_m[rayIdx][clusterIdx];

							double v =travelSpeed*cos((arr_angle - travelAngle)*PI / 180.) / Lambda;
							complex<double> F_rx_v = antennaMs->getominaVerticalFieldPattern(arr_angle, elevationtemp, u);
							complex<double> F_tx_v = antennaAP->getHeVerticalFieldPattern(dep_angle, elevationtemp, s);
							complex <double> exp_vv(0, Wifi_medi.at(N).initial_phase_vv_m[rayIdx][clusterIdx]);
							complex <double> j2pivt(0, 2.*PI*v*TTItime*0.001);
							complex <double> exp_d_u(0, d_u * 2 * PI / Lambda*sin(arr_angle * PI / 180.));
							complex <double> exp_d_s(0, d_s * 2 * PI / Lambda*sin(dep_angle * PI / 180.));
							double xpr = Wifi_medi.at(N).XPR_m[rayIdx][clusterIdx];
							complex<double> F_tx_h = antennaAP->getHeHorizontalFieldPattern(dep_angle, elevationtemp, s);
							complex<double> F_rx_h = antennaMs->getominaHorizontalFieldPattern(arr_angle, elevationtemp, u);
							complex <double> exp_vh(0, Wifi_medi.at(N).initial_phase_vh_m[rayIdx][clusterIdx]);
							complex <double> exp_hv(0, Wifi_medi.at(N).initial_phase_hv_m[rayIdx][clusterIdx]);
							complex <double> exp_hh(0, Wifi_medi.at(N).initial_phase_hh_m[rayIdx][clusterIdx]);
							//矩阵运算
							channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
								(F_tx_v*(F_rx_v*exp(exp_vv) + F_rx_h*exp(exp_hv)*sqrt(1 / xpr)) +
								F_tx_h*(F_rx_v*exp(exp_vh)*sqrt(1 / xpr) + F_rx_h*exp(exp_hh)))*exp(exp_d_s + exp_d_u + j2pivt);
						}

//做个标志				channel_temp[clusterIdx][u][s] *= sqrt(power_n);
//						channel_temp[clusterIdx][u][s] *= sqrt(power_n / Wifi_medi.at(N).num_rays_m[clusterIdx]);
						if (LOSflag == 1)
						{
							channel_temp[clusterIdx][u][s] *= sqrt(1 / (K + 1));
							if (clusterIdx == 0)
							{
								complex<double> F_rx_v_LOS = antennaMs->getominaVerticalFieldPattern(phi_AoA_LOS, elevationtemp, u);
								complex<double> F_tx_v_LOS = antennaAP->getHeVerticalFieldPattern(phi_AoD_LOS, elevationtemp, s);
								double v_LOS =travelSpeed*cos((phi_AoA_LOS - travelAngle)*PI / 180.) / Lambda;
								complex <double> exp_vv_LOS(0, Wifi_medi.at(N).initial_phase_vv_LOS_m);
								complex <double> j2pivt_LOS(0, 2.*PI*v_LOS*TTItime*0.001);
								complex <double> exp_d_u_LOS(0, d_u * 2 * PI / Lambda*sin(phi_AoA_LOS * PI / 180.));
								complex <double> exp_d_s_LOS(0, d_s * 2 * PI / Lambda*sin(phi_AoD_LOS * PI / 180.));
								complex<double> F_tx_h_LOS = antennaAP->getHeHorizontalFieldPattern(phi_AoD_LOS, elevationtemp, s);
								complex<double> F_rx_h_LOS = antennaMs->getominaHorizontalFieldPattern(phi_AoA_LOS, elevationtemp, u);
								complex <double> exp_hh_LOS(0, Wifi_medi.at(N).initial_phase_hh_LOS_m);
								channel_temp[clusterIdx][u][s] = channel_temp[clusterIdx][u][s] +
									sqrt(K / (K + 1))*((F_tx_v_LOS*F_rx_v_LOS*exp(exp_vv_LOS) + F_tx_h_LOS* F_rx_h_LOS*exp(exp_hh_LOS)))*exp(exp_d_s_LOS + exp_d_u_LOS + j2pivt_LOS);
							}
						}
					}
				}
			}
			double pathloss = Wifi_medi.at(N).pathLoss_m;
			double shadow = Wifi_medi.at(N).shadowFading_m;

			for (int j = 0; j < LN_size; j++)
			{
				for (int s = 0; s < Nt; s++)
				{
					for (int u = 0; u < Nr; u++)
					{
						channel_coefficients_time[u][s][j] = channel_temp[j][u][s];
						//channel_coefficients_time[s][u][j] = channel_temp[j][u][s];
					}
				}
			}
			/**************************************************************/
			//以下为做FFT变换，把时域信道系数转换为频域信道系数
			//generateFFTChannel();
			complex<double>* fft_in = new complex<double>[fftSize];
			complex<double>* fft_out = new complex<double>[fftSize];//
			complex<double>  channel_APcoefficients_freq[Nt][Nr][RBs_FOR_WIFI] = { complex<double>(0, 0) };       //====AP频域信道响应系数中间矩阵【基站天线】【用户天线】【子载波数】
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
						k = int(Wifi_medi.at(N).delay_clusters_m[i] / sample_time + 0.5);      //四舍五入，原来是向下取整
						if (k < fftSize)
						{
							fft_in[k] = fft_in[k] + channel_coefficients_time[u][s][i];
						}
					}

					FFTCalculation(fft_in, fft_out, fftSize);
					//做完FFT之后取点，顺序为：后取128个，前取128个，去掉零频
					//后取128点
					for (int k = 0; k< RBs_FOR_WIFI / 2; k++)
					{
						channel_APcoefficients_freq[u][s][k] = fft_out[fftSize - RBs_FOR_WIFI / 2 + k];
					}
					//前去128点，去掉零频
					for (int k = RBs_FOR_WIFI / 2; k < RBs_FOR_WIFI; k++)
					{
						channel_APcoefficients_freq[u][s][k] = fft_out[k - RBs_FOR_WIFI / 2 + 1];
					}
				}
			}
		
			//小尺度信道系数一维数组分配空间和初始化
			for (int i = 0; i < RBs_FOR_WIFI; i++)
			{
				  complex<double> temp_array[Nt*Nr] = { 0.0 };
				  for (int j = 0; j < Nt*Nr; j++)
					  temp_array[j] = channel_APcoefficients_freq[j %Nr][j / Nr][i];

				Matrix< complex<double>, Nr, Nt> B(temp_array);        //写成矩阵形式,每个频点上的H				 
				channel_APcoefficients[N][i]=B;
			}


			//释放内存空间
			delete[]fft_in;
			delete[]fft_out;
			delete antennaAP;
			delete antennaMs;
		}
	}



void ChannelPort::Compute_Ksi_eNodeBpathloss()               //只根据Ksi来更新大尺度
{
	for (int i = 0; i < nb_cell; i++)
	{


		eNodeB_power_loss[nb_sector * i] = eNodeB_power_loss[nb_sector * i] + eNodeB_shadow_loss[i];
		eNodeB_power_loss[nb_sector * i + 1] = eNodeB_power_loss[nb_sector * i + 1] + eNodeB_shadow_loss[i];
		eNodeB_power_loss[nb_sector * i + 2] = eNodeB_power_loss[nb_sector * i + 2] + eNodeB_shadow_loss[i];  //所有扇区
		 
		char* ch;
		if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)
			ch = "UMi";
		if (strcmp(SCENARIO_type, "TC1") == 0)
			ch = "TC1";
		generateLSPs(ch, eNodeB_isLOS[i], eNodeB_LSPs[i], NULL);
		eNodeB_shadow_loss[i] = 10.*log10(eNodeB_LSPs[i][nb_sector]);                          //重新计算阴影衰落，并依次重算大尺度eNodeB_Ksi[i]

		eNodeB_power_loss[nb_sector* i] = eNodeB_power_loss[nb_sector * i] - eNodeB_shadow_loss[i];
		eNodeB_power_loss[nb_sector * i + 1] = eNodeB_power_loss[nb_sector * i + 1] - eNodeB_shadow_loss[i];
		eNodeB_power_loss[nb_sector * i + 2] = eNodeB_power_loss[nb_sector * i + 2] - eNodeB_shadow_loss[i];  //所有扇区

	}

	for (int i = 0; i <nb_cell; i++)
	{

	}

}



void ChannelPort::Compute_Ksi_HeNodeBpathloss()
{
   
	for (int i = 0; i < nb_totalHenb; i++)
	{
		char* ch;
		if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)
			ch = "UMi";
		if (strcmp(SCENARIO_type, "TC1") == 0)
			ch = "TC1";

		HeNodeB_power_loss[i] = HeNodeB_power_loss[i] + HeNodeB_shadow_loss[i];

		generateLSPs(ch, HeNodeB_isLOS[i], HeNodeB_LSPs[i],NULL );               //重新计算阴影衰落，并依次重算大尺度HeNodeB_Ksi[i]

		HeNodeB_shadow_loss[i] = 10.*log10(HeNodeB_LSPs[i][nb_sector]);

		HeNodeB_power_loss[i] = HeNodeB_power_loss[i] - HeNodeB_shadow_loss[i];
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

