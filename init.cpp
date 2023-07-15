#include "init.h"
#include <cstring>
#include "iostream"
#include <cmath>
#include "ENodeB.h"
#include "HeNodeB.h"
#include "AP.h"
#include "UserEquipment.h"
#include "position.h"
#include "NetworkManager.h"
#include "NetworkNode.h"
#include "ap-wifi-phy.h"
#include "wifi-bandwidth-manager.h"
#include "lte-bandwidth-manager.h"
#include "enb-lte-phy.h"
#include "henb-lte-phy.h"
#include "time.h"
#include "parameter.h"
#include "application.h"
#include "Mobility.h"
#include  "Building.h"


//半径为500米的小区中心
static double cellPosition[19][2] =
{
	{ 0., 0. },				    // 0
	{ 0., 866. },				// 1
	{ -750., 433. },			// 2
	{ -750, -433 },			    // 3
	{ 0., -866. },				// 4
	{ 750., -433. },			// 5
	{ 750., 433. },			    // 6	
	{ 0., 1732. },				// 7
	{ -750., 1299. },			// 8
	{ -1500., 866. },			// 9
	{ -1500., 0. },				// 10
	{ -1500., -866. },			// 11
	{ -750., -1299. },			// 12
	{ 0., -1732 },				// 13
	{ 750., -1299. },			// 14
	{ 1500., -866. },			// 15
	{ 1500., 0 },				// 16
	{ 1500., 866. },			// 17
	{ 750., 1299. },			// 18
};


static double SectorPosition[19 * 3][2] =
{

	{ 250, 144 },
	{ -250, 144 },                          //对应57个扇区的中心位置
	{ 0, -289 },


	{ 250, 1010 },
	{ -250, 1010 },
	{ 0, 577 },

	{ -500, 577 },
	{ -1000, 577 },
	{ -750, 144 },


	{ -500, -289 },
	{ -1000, -289 },
	{ -750, -722 },


	{ 250, -722 },
	{ -250, -722 },
	{ 0, -1155 },

	{ 1000, -289 },
	{ 500, -289 },
	{ 750, -722 },


	{ 1000, 577 },
	{ 500, 577 },
	{ 750, 144 },

	{ 250, 1876 },
	{ -250, 1876 },
	{ 0, 1443 },


	{ -500, 1443 },
	{ -1000, 1443 },
	{ -750, 1010 },


	{ -1250, 1010 },
	{ -1750, 1010 },
	{ -1500, 577 },

	{ -1250, 144 },
	{ -1750, 144 },
	{ -1500, -289 },


	{ -1250, -722 },
	{ -1750, -722 },
	{ -1500, -1155 },


	{ -500, -1155 },
	{ -1000, -1155 },
	{ -750, -1588 },

	{ 250, -1588 },
	{ -250, -1588 },
	{ 0, -2021 },

	{ 1000, -1155 },
	{ 500, -1155 },
	{ 750, -1588 },

	{ 1750, -722 },
	{ 1250, -722 },
	{ 1500, -1155 },

	{ 1750, 144 },
	{ 1250, 144 },
	{ 1500, -289 },

	{ 1750, 1010 },
	{ 1250, 1010 },
	{ 1500, 577 },

	{ 1000, 1443 },
	{ 500, 1443 },
	{ 750, 1010 },
};


Position* 
ap_distribute(Position * ps, double rd)        //在中心位置为ps,半径为rd的圆内产生一个随机位置
{
	double x, y, m, n;
	//	srand((unsigned)time(NULL));
	m = (double)rand() / (0x7fff)*rd;
	n = (double)rand() / (0x7fff)* 2 * PI;
	x = ps->GetPositionX() + m*cos(n);
	y = ps->GetPositionY() + m*sin(n);;
	Position* pos = new Position(x, y,ps->GetPositionZ());
	return pos;
}

Position*
hexagon_distribute(Position * center_pos,double rd)        //在中心位置为ps的扇区六边形内利用割补法产生一个随机位置
{
	double 	Y = ((double)rand() / 0x7fff*1.5 - 1)*rd / sqrt(3);
	double 	X = ((double)rand() / 0x7fff * 2 - 1)*rd / 2;        //[-0.5r,0.5r]
	if (Y + sqrt(3.0) / 3 * X + rd / sqrt(3) < 0)                //判断是否处于要平移的左下三角形
	{
		X = X + rd / 2;
		Y = Y + 3 / 2 * rd / sqrt(3);
	}
	else if (Y - sqrt(3.0) / 3 * X + rd / sqrt(3) < 0)          //判断是否处于要平移的右下三角形   
	{
		X = X - rd / 2;
		Y = Y + 3 / 2 * rd / sqrt(3);
	}
	X = X + center_pos->GetPositionX();         //对应不同扇区的真实位置
	Y = Y + center_pos->GetPositionY();

	Position* pos = new Position(X, Y, center_pos->GetPositionZ());
	return pos;

}



Init::Init()
{}


Init::~Init()
{}

void 
Init::Net_Init()    //根据场景进行拓扑生成和撒点,包括三大场景
{

	if (SCENARIO == 1)               // 第1种场景：UDN-室外和室内，室外包括UMi、UMa、RMa、SMa
	{
		NetworkManager* nm = NetworkManager::Init();

		if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)     //室外场景
		{
			double ISD;                //基站间间隔
			double Min_distance;       //基站最小距离限制
			double Bs_height;          //宏基站天线高度（小基站一律为10m）

			if (strcmp(SCENARIO_type, "UMi") == 0)     //UMi
			{
				ISD = 200;
				Min_distance = 10;		
				Bs_height = 10;
			}

			if (strcmp(SCENARIO_type, "UMa") == 0)   //UMa
			{
				ISD = 500;
				Min_distance = 25;
				Bs_height = 25;
			}

			if (strcmp(SCENARIO_type, "RMa") == 0)  //RMa
			{
				ISD = 1732;
				Min_distance = 35;
				Bs_height = 35;


			}

			if (strcmp(SCENARIO_type, "SMa") == 0)  //SMa
			{
				ISD = 1299;
				Min_distance = 35;
				Bs_height = 35;
			}

			//室外场景按照六边形拓扑进行初始化，已存基站位置按照小区半径为500m进行参考分布的，在计算实际半径后进行调整

			double radius = ISD / sqrt(3);                   //这里的radius为扇区中心间隔(半径)，ISD为小区中心间隔
			srand((unsigned int)time(NULL));


			std::cout << "仿真场景：" << SCENARIO_type << endl;

	
			//生成基站,基站号和扇区号对应
			std::cout << "生成基站" << endl;

			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double X = radius / 500 * cellPosition[i][0];
				double Y = radius / 500 * cellPosition[i][1];
				double Z = Bs_height;
				Position *enb_pos = new Position(X, Y, Z);
				ENodeB* enb = new ENodeB(i, enb_pos);
				enb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));       //带宽
				enb->SetPostate(m_Outdoor); //室外
				nm->GetENodeBContainer()->push_back(enb);

				//信息检验输出：
		//		enb->GetLtePhy()->GetlteBandwidthManager()->Print();
				std::cout << "Created enb, id " << enb->GetIDNetworkNode()
					<< ", position: " << enb->Getposition()->GetPositionX()
					<< " " << enb->Getposition()->GetPositionY() << endl;
			}

			std::cout << "生成小基站" << endl;
			int id_Henbs = 0;                                       //小基站的编号
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 10;
				Position *s = new Position(x, y, z);        //扇区中心

				for (int j = 0; j < nb_HeNodeB_sector; j++)
				{
					Position* Heb_pos = hexagon_distribute(s, radius);
					HeNodeB* Heb = new HeNodeB(id_Henbs, i, Heb_pos); //生成小基站
					Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //带宽
					Heb->SetPostate(m_Outdoor);  
					nm->GetHeNodeBContainer()->push_back(Heb);
					id_Henbs++;

					//信息检验输出：
				//    Heb->GetLtePhy()->GetlteBandwidthManager()->Print();
					std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
						<< ", position: " << Heb->Getposition()->GetPositionX()
						<< " " << Heb->Getposition()->GetPositionY() << endl;

				}
				delete s;
			}


			std::cout << "生成wifi-AP" << endl;

			int id_ap = 0;      //ap的编号
			//在每个扇区中利用割补法产生AP群中心，并且以群中心放置5个wifi
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 1.5;
				Position *s = new Position(x, y, z);        //扇区中心

			    for (int j = 0; j < nb_apgrp; j++)
			   {
				  Position* grp_center = hexagon_distribute(s, radius);     //AP群中心

					for (int k = 0; k < nb_apingroup; k++)
					{
						Position*pos = ap_distribute(grp_center, ap_radius);     //以中心点为半径产生ap的位置
						AP* ap = new AP(id_ap, i, pos);        
						ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, k));
						ap->SetPostate(m_Outdoor);  
						nm->GetAPContainer()->push_back(ap);
						id_ap++;

                     //信息检验输出：
					//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
						std::cout << "Created AP, id " << ap->GetIDNetworkNode()
							<< ",sector id" << ap->GetIndex_sector()
							<< ", position: " << ap->Getposition()->GetPositionX()
							<< " " << ap->Getposition()->GetPositionY() << endl;
					}
				}
				delete s;
			}


			std::cout << "生成用户" << endl;
			int id_ue = 0;                                 //ue的编号
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 1.5;
				Position *s = new Position(x, y, z);        //扇区中心

				for (int j = 0; j < nb_ue_sector; j++)
				{
					
					Positionstate t;
					Position* Ue_pos = hexagon_distribute(s, radius);


					do{ Position* Ue_pos = hexagon_distribute(s, radius); }
					while 
					(sqrt(pow(nm->GetENodeBContainer()->at(i)->Getposition()->GetPositionX() - Ue_pos->GetPositionX(), 2) +
					pow(nm->GetENodeBContainer()->at(3)->Getposition()->GetPositionY() - Ue_pos->GetPositionY(), 2)) < Min_distance);  //最小距离限制:水平面

					if (strcmp(SCENARIO_type, "UMi") == 0)
					{
					double Rn = (double)rand() / 0x7fff;
					if (Rn > 0.5)
						t = m_Outdoor;               //50% users outdoor(pedestrian users) and 50% of users indoors
					else
						t = m_Indoor;
					}

					if (strcmp(SCENARIO_type, "UMa") == 0)
					{
						t = m_Vehicle;               //100% of users outdoors in vehicles
					}

					if (strcmp(SCENARIO_type, "RMa") == 0)
					{
						t = m_Vehicle;               //100% of users outdoors in high speed vehicles
					}

					if (strcmp(SCENARIO_type, "SMa") == 0)
					{
						double Rn = (double)rand() / 0x7fff;
						if (Rn > 0.5)
							t = m_Vehicle;          //50% users vehicles and 50% of users indoors
						else
							t = m_Indoor;
					}
					UserEquipment* user = new UserEquipment(id_ue, Ue_pos, i, NULL, NULL, NULL, new Mobility(SCENARIO_type,t,1));
					user->SetPostate(t);

					nm->GetUserEquipmentContainer()->push_back(user);
					id_ue++;

					Application *  App_1 = new Application(Application::ApplicationType::TYPE_VOIP, 0, Sim_stop_time); //用户生成业务
					user->GetApplist()->push_back(App_1);       //生成业务   最多2个业务       
					user->NetSelection();					    //业务选网 LTE或者WIFI ,   依次确定发送队列

					user->wifi_link_update();                   //更新信道
					user->lte_link_update();


					std::cout << "user's, id " << user->GetIDNetworkNode()
						<< ",sector id" << user->GetIndex_sector()
						<< ", position: " << user->Getposition()->GetPositionX()
						<< " " << user->Getposition()->GetPositionY() << "  ";
					cout << "target_ap:" << user->GetTargetAPNode()->GetIDNetworkNode() <<endl;
					if (user->GetTargetEnbNode() != NULL)
						cout << "target_EnbNode:" << user->GetTargetEnbNode()->GetIDNetworkNode() << endl;
					else if (user->GetTargetHenbNode() != NULL)
						cout << "target_HenbNode:" << user->GetTargetHenbNode()->GetIDNetworkNode() << endl;


					//list<Application::flowNode *>* App_datalist = user->GetApplist()->at(0)->GetDatalist();
					//list<Application::flowNode *>::iterator it;
					//cout << "用户" << user->GetIDNetworkNode() << "的业务流：" << endl;
					//for (it = App_datalist->begin(); it != App_datalist->end(); it++)
					//{
					//	cout << "数据包大小:" << (*it)->m_data_size << ",到达时间：" << (*it)->m_data_arrive_time << endl;
					//}		
				}
			}
		}


		if (strcmp(SCENARIO_type, "TC1") == 0)     //室内  20×10的办公司
		{
			std::cout << "仿真场景：真实室内办公室" << endl;

			std::cout << "生成小基站" << endl;
			double 	HeNodeB_X = 10;
			double 	HeNodeB_Y = 5;
			double  HeNodeB_Z = 2.9;
			HeNodeB* Heb = new HeNodeB(0, 0, new Position(HeNodeB_X, HeNodeB_Y, HeNodeB_Z));       //生成小基站(1个)
			Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //带宽
			Heb->SetPostate(m_Indoor);

			nm->GetHeNodeBContainer()->push_back(Heb);
			//信息检验输出：
			//	Heb->GetLtePhy()->GetlteBandwidthManager()->Print();
			std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
				<< ", position: " << Heb->Getposition()->GetPositionX()
				<< ", " << Heb->Getposition()->GetPositionY() << endl;


			std::cout << "生成wifi-AP" << endl;
			double m_interval = 20.0 / double(nb_totalAP);
			//在房间内随机分布5个wifi
			for (int i = 0; i < nb_totalAP; i++)
			{
				double	AP_center_X = ((double)rand() / 0x7fff + i)*m_interval;
				double  AP_center_Y = ((double)rand() / 0x7fff) * 10;                     //[0,10]     
				double  AP_center_Z = 1.5;
				AP* ap = new AP(i, 0, new Position(AP_center_X, AP_center_Y, AP_center_Y));   //AP的分布
				ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, i));
				ap->SetPostate(m_Indoor);

				nm->GetAPContainer()->push_back(ap);

				//信息检验输出：
				//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
				std::cout << "Created AP, id " << ap->GetIDNetworkNode()
					<< ", position: " << ap->Getposition()->GetPositionX()
					<< ", " << ap->Getposition()->GetPositionY() << endl;
			}


			std::cout << "生成用户" << endl;

			for (int i = 0; i < nb_totalUe; i++)                    //ue均匀分布在2.9×20×10空间内
			{
				double	UE_X = ((double)rand() / 0x7fff) * 20;
				double	UE_Y = ((double)rand() / 0x7fff) * 10;
				double	UE_Z = 1.5;
				UserEquipment* user = new UserEquipment(i, new Position(UE_X, UE_Y, UE_Z), 0, NULL, NULL, NULL, new Mobility(SCENARIO_type, m_Indoor, 1));
				user->SetPostate(m_Indoor);

				nm->GetUserEquipmentContainer()->push_back(user);

				Application *  App_1 = new Application(Application::ApplicationType::TYPE_FULLBUFFER, 0, Sim_stop_time); //用户生成业务				
				user->GetApplist()->push_back(App_1);                              //生成业务   最多2个业务       		
				user->NetSelection();                                  	           //业务选网 LTE或者WIFI 
				user->wifi_link_update();                                           //根据位置进行距离、天线增益，连接目标等更新
				user->lte_link_update();

				std::cout << "Created user, id " << user->GetIDNetworkNode()
					<< ",sector id" << user->GetIndex_sector()
					<< ", position: " << user->Getposition()->GetPositionX()
					<< " " << user->Getposition()->GetPositionY() << endl;
				//		cout << "target_ap:" << user->GetTargetAPNode()->GetIDNetworkNode() << "  ";

			}
		}



		if (strcmp(SCENARIO_type, "TC2") == 0)     //
		{
			std::cout << "仿真场景：密集都市信息社会" << endl;
			double center_pos[4][2] = { { 69, 69 }, { -69, 69 }, { -69, -69 }, { 69, -69 } }; //大楼中心位置
			std::cout << "生成大楼" << endl;                 //to see METIS D6.1_v1 Table 3.7
			for (int i = 0; i < 4; i++)
			{
				Building*bd = new Building(i, 6, 3.5, 60, 60, center_pos[i][0], center_pos[i][1]);
				nm->GetBuildingContainer()->push_back(bd);
			}

			std::cout << "生成宏基站" << endl;			
			Position* enb_pos = new Position(69, 69, 26);   //26=3.5*6+5,位于顶层5m处
			for (int i = 0; i < nb_totalEnb ; i++)
			{
				ENodeB* enb = new ENodeB(i, enb_pos);
				enb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));      
				enb->SetPostate(m_Outdoor);                                                      
				nm->GetENodeBContainer()->push_back(enb);

				//信息检验输出：		
				std::cout << "Created enb, id " << enb->GetIDNetworkNode()
					<< ", position: " << enb->Getposition()->GetPositionX()
					<< ", " << enb->Getposition()->GetPositionY() 
					<< " " << enb->Getposition()->GetPositionZ() << endl;
			}

			std::cout << "生成小基站" << endl;
			double henb_pos[4][2] = { { -6, 69 }, { -69, 6 }, { 6, -69 }, { 69, -6 } }; //小基站平面位置
			for (int i = 0; i < nb_totalHenb; i++)
			{
				Position* enb_pos = new Position(henb_pos[i][0], henb_pos[i][1], 10);
				HeNodeB* Heb = new HeNodeB(i, 0, enb_pos);
			    Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //带宽
			    Heb->SetPostate(m_Outdoor);
			    nm->GetHeNodeBContainer()->push_back(Heb);

				//信息检验输出：
				std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
					<< ", position: " << Heb->Getposition()->GetPositionX()
					<< ", " << Heb->Getposition()->GetPositionY() 
					<< ", " << Heb->Getposition()->GetPositionZ() << endl;

			}

			std::cout << "生成wifi-AP" << endl;
			double Evenfloor_ap_pos[10][2] = { { -40, 36 }, { 0, 36 }, { 40, 36 }, { -20, 12 }, { 20, 12 }, { -40, -12 }, { 0, -12 }, { 40, -12 }, { -20, -36 }, {20,-36} }; //AP position of different floor
			double Oddfloor_ap_pos[10][2] = { { -20, 36 }, { 20, 36 }, { -40, 12 }, { 0, 12 }, { 40, 12 }, { -20, -12 }, { 20, -12 }, { -40, -36 }, { 0, -36 }, { 40, -36 } };
			int ID = 0;
			for (int i = 0; i < 4; i++)
			{
				double x = center_pos[i][0];
				double y = center_pos[i][1];              //所属大楼
				double	AP_X, AP_Y, AP_Z;
				for (int j = 0; j < nb_totalAP/4; j++)          //10 APs in each floor
				{
					int floor = j / (nb_totalAP / 4/6);         //一栋大楼6层，一共4座大楼
					if (j % 2 == 0)
					 {
						AP_X = Evenfloor_ap_pos[j % 10][0] + x;
						AP_Y = Evenfloor_ap_pos[j % 10][1] + y;
						AP_Z = floor*3.5 + 3;
					 }
					else
					{
						AP_X = Oddfloor_ap_pos[j % 10][0] + x;
						AP_Y = Oddfloor_ap_pos[j % 10][1] + y;
						AP_Z = floor*3.5 + 3;
					}
					AP* ap = new AP(ID, 0, new Position(AP_X, AP_Y, AP_Z));   //AP的分布
					ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, j));
					ap->SetPostate(m_Indoor);
					nm->GetAPContainer()->push_back(ap);
					ID++;
					//信息检验输出：
					//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
					std::cout << "Created AP, id " << ap->GetIDNetworkNode()
						<< ", position: " << ap->Getposition()->GetPositionX()
						<< ", " << ap->Getposition()->GetPositionY() 
						<< ", " << ap->Getposition()->GetPositionZ() << endl;

				}
			 }

			std::cout << "生成用户" << endl;
			ID = 0;
			double  UE_X, UE_Y, UE_Z;
            //室内分布：10个用户随机分布在每一层大楼内,共240人
			  for (int i = 0; i < 4; i++)         
			  {
				  double x = center_pos[i][0];
				  double y = center_pos[i][1];              //所属大楼
				  for (int j = 0; j < 6*Ue_Perfloor; j++)          //10 UEs in each floor               Ue_Perfloor             
				  {
					  int floor = j / Ue_Perfloor;
					  UE_X = ((double)rand() / 0x7fff * 2 - 1) * 50 + x;
					  UE_Y = ((double)rand() / 0x7fff * 2 - 1) * 50 + y; //    
					  UE_Z = floor*3.5 + 1.5;
					  UserEquipment* user = new UserEquipment(ID, new Position(UE_X, UE_Y, UE_Z), 0, NULL, NULL, NULL, new Mobility(SCENARIO_type, m_Indoor, 1));
					  user->SetPostate(m_Indoor);
					  nm->GetUserEquipmentContainer()->push_back(user);
					  ID++;
				  }
			  }
     
			//室外分布：随机分布60人，分布在两条主干道上
			  for (int i = 0; i < nb_totalUe - 4 * 6 * Ue_Perfloor; i++)
			  {
				  Mobility * Mob = new Mobility(SCENARIO_type, m_Outdoor, 2);   //实际移动

				  if (i < (nb_totalUe - 4 * 6 * Ue_Perfloor) / 2)
				  {
				    UE_X = ((double)rand() / 0x7fff * 2 - 1) * 138;   //x【-138，138】       
				    UE_Y = ((double)rand() / 0x7fff * 2 - 1) * 9 ;    // y【-9，9】 
					UE_Z = 1.5;
				    Mob->m_speedDirection = ((double)rand() / 0x7fff * 2 - 1)>0 ? 0 : 180;    // 东西走向的用户移动方向为正东或正西 (随机取)
					if (UE_X>69)
						Mob->m_speedDirection = 180;
					else if (UE_X<-69)
						Mob->m_speedDirection = 0;                   //保证仿真期间不出边界
				  }
				   else
				  {
					 double  UE_X = ((double)rand() / 0x7fff * 2 - 1) * 9;     // x【-9，9】
					 double  UE_Y = ((double)rand() / 0x7fff * 2 - 1) * 138;   //y【-138，138】
					 UE_Z = 1.5;
					 Mob->m_speedDirection = ((double)rand() / 0x7fff * 2 - 1)>0 ? 90 : -90;    // 南北走向的用户移动方向为正南或正北 (随机取)
					 if (UE_Y>69)
						 Mob->m_speedDirection = -90;
					 else if (UE_Y<-69)
						 Mob->m_speedDirection = 90;                   //保证仿真期间不出边界
				  }

				  UserEquipment* user = new UserEquipment(ID, new Position(UE_X, UE_Y, UE_Z), 0, NULL, NULL, NULL,Mob);
				  user->SetPostate(m_Outdoor);
				  nm->GetUserEquipmentContainer()->push_back(user);
				  ID++;
			  }

			  //以下此处设定用户业务以及通信模式（D2D模式或者蜂窝模式）
			  for (int i = 0; i < nb_totalUe; i++)
			  {
				  UserEquipment* user = nm->GetUserEquipmentContainer()->at(i);		 
				  Application *  App_1 = new Application(Application::ApplicationType::TYPE_FULLBUFFER, 0, Sim_stop_time); //用户生成业务
				  user->GetApplist()->push_back(App_1);       //生成业务   最多2个业务       
				  user->NetSelection();					    //业务选网 LTE或者WIFI ,   依次确定发送队列
				  user->wifi_link_update();                   //更新信道
				  user->lte_link_update();
              
				std::cout << "Created user, id " << user->GetIDNetworkNode()
					<< ",sector id" << user->GetIndex_sector()
					<< ", position: " << user->Getposition()->GetPositionX()
					<< ", " << user->Getposition()->GetPositionY()
				    << ", " << user->Getposition()->GetPositionZ() << endl;
					//cout << "target_ap:" << user->GetTargetAPNode()->GetIDNetworkNode() << "  ";
              }
		}
	}

	if (SCENARIO == 2){}         //第二类场景


	if (SCENARIO == 3){}         //第三类场景


		
}






