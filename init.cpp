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


//�뾶Ϊ500�׵�С������
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
	{ -250, 144 },                          //��Ӧ57������������λ��
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
ap_distribute(Position * ps, double rd)        //������λ��Ϊps,�뾶Ϊrd��Բ�ڲ���һ�����λ��
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
hexagon_distribute(Position * center_pos,double rd)        //������λ��Ϊps�����������������ø������һ�����λ��
{
	double 	Y = ((double)rand() / 0x7fff*1.5 - 1)*rd / sqrt(3);
	double 	X = ((double)rand() / 0x7fff * 2 - 1)*rd / 2;        //[-0.5r,0.5r]
	if (Y + sqrt(3.0) / 3 * X + rd / sqrt(3) < 0)                //�ж��Ƿ���Ҫƽ�Ƶ�����������
	{
		X = X + rd / 2;
		Y = Y + 3 / 2 * rd / sqrt(3);
	}
	else if (Y - sqrt(3.0) / 3 * X + rd / sqrt(3) < 0)          //�ж��Ƿ���Ҫƽ�Ƶ�����������   
	{
		X = X - rd / 2;
		Y = Y + 3 / 2 * rd / sqrt(3);
	}
	X = X + center_pos->GetPositionX();         //��Ӧ��ͬ��������ʵλ��
	Y = Y + center_pos->GetPositionY();

	Position* pos = new Position(X, Y, center_pos->GetPositionZ());
	return pos;

}



Init::Init()
{}


Init::~Init()
{}

void 
Init::Net_Init()    //���ݳ��������������ɺ�����,�������󳡾�
{

	if (SCENARIO == 1)               // ��1�ֳ�����UDN-��������ڣ��������UMi��UMa��RMa��SMa
	{
		NetworkManager* nm = NetworkManager::Init();

		if (strcmp(SCENARIO_type, "UMi") == 0 || strcmp(SCENARIO_type, "UMa") == 0 || strcmp(SCENARIO_type, "RMa") == 0 || strcmp(SCENARIO_type, "SMa") == 0)     //���ⳡ��
		{
			double ISD;                //��վ����
			double Min_distance;       //��վ��С��������
			double Bs_height;          //���վ���߸߶ȣ�С��վһ��Ϊ10m��

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

			//���ⳡ���������������˽��г�ʼ�����Ѵ��վλ�ð���С���뾶Ϊ500m���вο��ֲ��ģ��ڼ���ʵ�ʰ뾶����е���

			double radius = ISD / sqrt(3);                   //�����radiusΪ�������ļ��(�뾶)��ISDΪС�����ļ��
			srand((unsigned int)time(NULL));


			std::cout << "���泡����" << SCENARIO_type << endl;

	
			//���ɻ�վ,��վ�ź������Ŷ�Ӧ
			std::cout << "���ɻ�վ" << endl;

			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double X = radius / 500 * cellPosition[i][0];
				double Y = radius / 500 * cellPosition[i][1];
				double Z = Bs_height;
				Position *enb_pos = new Position(X, Y, Z);
				ENodeB* enb = new ENodeB(i, enb_pos);
				enb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));       //����
				enb->SetPostate(m_Outdoor); //����
				nm->GetENodeBContainer()->push_back(enb);

				//��Ϣ���������
		//		enb->GetLtePhy()->GetlteBandwidthManager()->Print();
				std::cout << "Created enb, id " << enb->GetIDNetworkNode()
					<< ", position: " << enb->Getposition()->GetPositionX()
					<< " " << enb->Getposition()->GetPositionY() << endl;
			}

			std::cout << "����С��վ" << endl;
			int id_Henbs = 0;                                       //С��վ�ı��
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 10;
				Position *s = new Position(x, y, z);        //��������

				for (int j = 0; j < nb_HeNodeB_sector; j++)
				{
					Position* Heb_pos = hexagon_distribute(s, radius);
					HeNodeB* Heb = new HeNodeB(id_Henbs, i, Heb_pos); //����С��վ
					Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //����
					Heb->SetPostate(m_Outdoor);  
					nm->GetHeNodeBContainer()->push_back(Heb);
					id_Henbs++;

					//��Ϣ���������
				//    Heb->GetLtePhy()->GetlteBandwidthManager()->Print();
					std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
						<< ", position: " << Heb->Getposition()->GetPositionX()
						<< " " << Heb->Getposition()->GetPositionY() << endl;

				}
				delete s;
			}


			std::cout << "����wifi-AP" << endl;

			int id_ap = 0;      //ap�ı��
			//��ÿ�����������ø������APȺ���ģ�������Ⱥ���ķ���5��wifi
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 1.5;
				Position *s = new Position(x, y, z);        //��������

			    for (int j = 0; j < nb_apgrp; j++)
			   {
				  Position* grp_center = hexagon_distribute(s, radius);     //APȺ����

					for (int k = 0; k < nb_apingroup; k++)
					{
						Position*pos = ap_distribute(grp_center, ap_radius);     //�����ĵ�Ϊ�뾶����ap��λ��
						AP* ap = new AP(id_ap, i, pos);        
						ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, k));
						ap->SetPostate(m_Outdoor);  
						nm->GetAPContainer()->push_back(ap);
						id_ap++;

                     //��Ϣ���������
					//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
						std::cout << "Created AP, id " << ap->GetIDNetworkNode()
							<< ",sector id" << ap->GetIndex_sector()
							<< ", position: " << ap->Getposition()->GetPositionX()
							<< " " << ap->Getposition()->GetPositionY() << endl;
					}
				}
				delete s;
			}


			std::cout << "�����û�" << endl;
			int id_ue = 0;                                 //ue�ı��
			for (int i = 0; i < 3 * nb_cell; i++)
			{
				double x = radius / 500 * SectorPosition[i][0];
				double y = radius / 500 * SectorPosition[i][1];
				double z = 1.5;
				Position *s = new Position(x, y, z);        //��������

				for (int j = 0; j < nb_ue_sector; j++)
				{
					
					Positionstate t;
					Position* Ue_pos = hexagon_distribute(s, radius);


					do{ Position* Ue_pos = hexagon_distribute(s, radius); }
					while 
					(sqrt(pow(nm->GetENodeBContainer()->at(i)->Getposition()->GetPositionX() - Ue_pos->GetPositionX(), 2) +
					pow(nm->GetENodeBContainer()->at(3)->Getposition()->GetPositionY() - Ue_pos->GetPositionY(), 2)) < Min_distance);  //��С��������:ˮƽ��

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

					Application *  App_1 = new Application(Application::ApplicationType::TYPE_VOIP, 0, Sim_stop_time); //�û�����ҵ��
					user->GetApplist()->push_back(App_1);       //����ҵ��   ���2��ҵ��       
					user->NetSelection();					    //ҵ��ѡ�� LTE����WIFI ,   ����ȷ�����Ͷ���

					user->wifi_link_update();                   //�����ŵ�
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
					//cout << "�û�" << user->GetIDNetworkNode() << "��ҵ������" << endl;
					//for (it = App_datalist->begin(); it != App_datalist->end(); it++)
					//{
					//	cout << "���ݰ���С:" << (*it)->m_data_size << ",����ʱ�䣺" << (*it)->m_data_arrive_time << endl;
					//}		
				}
			}
		}


		if (strcmp(SCENARIO_type, "TC1") == 0)     //����  20��10�İ칫˾
		{
			std::cout << "���泡������ʵ���ڰ칫��" << endl;

			std::cout << "����С��վ" << endl;
			double 	HeNodeB_X = 10;
			double 	HeNodeB_Y = 5;
			double  HeNodeB_Z = 2.9;
			HeNodeB* Heb = new HeNodeB(0, 0, new Position(HeNodeB_X, HeNodeB_Y, HeNodeB_Z));       //����С��վ(1��)
			Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //����
			Heb->SetPostate(m_Indoor);

			nm->GetHeNodeBContainer()->push_back(Heb);
			//��Ϣ���������
			//	Heb->GetLtePhy()->GetlteBandwidthManager()->Print();
			std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
				<< ", position: " << Heb->Getposition()->GetPositionX()
				<< ", " << Heb->Getposition()->GetPositionY() << endl;


			std::cout << "����wifi-AP" << endl;
			double m_interval = 20.0 / double(nb_totalAP);
			//�ڷ���������ֲ�5��wifi
			for (int i = 0; i < nb_totalAP; i++)
			{
				double	AP_center_X = ((double)rand() / 0x7fff + i)*m_interval;
				double  AP_center_Y = ((double)rand() / 0x7fff) * 10;                     //[0,10]     
				double  AP_center_Z = 1.5;
				AP* ap = new AP(i, 0, new Position(AP_center_X, AP_center_Y, AP_center_Y));   //AP�ķֲ�
				ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, i));
				ap->SetPostate(m_Indoor);

				nm->GetAPContainer()->push_back(ap);

				//��Ϣ���������
				//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
				std::cout << "Created AP, id " << ap->GetIDNetworkNode()
					<< ", position: " << ap->Getposition()->GetPositionX()
					<< ", " << ap->Getposition()->GetPositionY() << endl;
			}


			std::cout << "�����û�" << endl;

			for (int i = 0; i < nb_totalUe; i++)                    //ue���ȷֲ���2.9��20��10�ռ���
			{
				double	UE_X = ((double)rand() / 0x7fff) * 20;
				double	UE_Y = ((double)rand() / 0x7fff) * 10;
				double	UE_Z = 1.5;
				UserEquipment* user = new UserEquipment(i, new Position(UE_X, UE_Y, UE_Z), 0, NULL, NULL, NULL, new Mobility(SCENARIO_type, m_Indoor, 1));
				user->SetPostate(m_Indoor);

				nm->GetUserEquipmentContainer()->push_back(user);

				Application *  App_1 = new Application(Application::ApplicationType::TYPE_FULLBUFFER, 0, Sim_stop_time); //�û�����ҵ��				
				user->GetApplist()->push_back(App_1);                              //����ҵ��   ���2��ҵ��       		
				user->NetSelection();                                  	           //ҵ��ѡ�� LTE����WIFI 
				user->wifi_link_update();                                           //����λ�ý��о��롢�������棬����Ŀ��ȸ���
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
			std::cout << "���泡�����ܼ�������Ϣ���" << endl;
			double center_pos[4][2] = { { 69, 69 }, { -69, 69 }, { -69, -69 }, { 69, -69 } }; //��¥����λ��
			std::cout << "���ɴ�¥" << endl;                 //to see METIS D6.1_v1 Table 3.7
			for (int i = 0; i < 4; i++)
			{
				Building*bd = new Building(i, 6, 3.5, 60, 60, center_pos[i][0], center_pos[i][1]);
				nm->GetBuildingContainer()->push_back(bd);
			}

			std::cout << "���ɺ��վ" << endl;			
			Position* enb_pos = new Position(69, 69, 26);   //26=3.5*6+5,λ�ڶ���5m��
			for (int i = 0; i < nb_totalEnb ; i++)
			{
				ENodeB* enb = new ENodeB(i, enb_pos);
				enb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));      
				enb->SetPostate(m_Outdoor);                                                      
				nm->GetENodeBContainer()->push_back(enb);

				//��Ϣ���������		
				std::cout << "Created enb, id " << enb->GetIDNetworkNode()
					<< ", position: " << enb->Getposition()->GetPositionX()
					<< ", " << enb->Getposition()->GetPositionY() 
					<< " " << enb->Getposition()->GetPositionZ() << endl;
			}

			std::cout << "����С��վ" << endl;
			double henb_pos[4][2] = { { -6, 69 }, { -69, 6 }, { 6, -69 }, { 69, -6 } }; //С��վƽ��λ��
			for (int i = 0; i < nb_totalHenb; i++)
			{
				Position* enb_pos = new Position(henb_pos[i][0], henb_pos[i][1], 10);
				HeNodeB* Heb = new HeNodeB(i, 0, enb_pos);
			    Heb->GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("henb"));       //����
			    Heb->SetPostate(m_Outdoor);
			    nm->GetHeNodeBContainer()->push_back(Heb);

				//��Ϣ���������
				std::cout << "Created Henb, id " << Heb->GetIDNetworkNode()
					<< ", position: " << Heb->Getposition()->GetPositionX()
					<< ", " << Heb->Getposition()->GetPositionY() 
					<< ", " << Heb->Getposition()->GetPositionZ() << endl;

			}

			std::cout << "����wifi-AP" << endl;
			double Evenfloor_ap_pos[10][2] = { { -40, 36 }, { 0, 36 }, { 40, 36 }, { -20, 12 }, { 20, 12 }, { -40, -12 }, { 0, -12 }, { 40, -12 }, { -20, -36 }, {20,-36} }; //AP position of different floor
			double Oddfloor_ap_pos[10][2] = { { -20, 36 }, { 20, 36 }, { -40, 12 }, { 0, 12 }, { 40, 12 }, { -20, -12 }, { 20, -12 }, { -40, -36 }, { 0, -36 }, { 40, -36 } };
			int ID = 0;
			for (int i = 0; i < 4; i++)
			{
				double x = center_pos[i][0];
				double y = center_pos[i][1];              //������¥
				double	AP_X, AP_Y, AP_Z;
				for (int j = 0; j < nb_totalAP/4; j++)          //10 APs in each floor
				{
					int floor = j / (nb_totalAP / 4/6);         //һ����¥6�㣬һ��4����¥
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
					AP* ap = new AP(ID, 0, new Position(AP_X, AP_Y, AP_Z));   //AP�ķֲ�
					ap->GetWifiPhy()->SetwifiBandwidthManager(new wifi_BandwidthManager(WIFI_BANDWIDTH, j));
					ap->SetPostate(m_Indoor);
					nm->GetAPContainer()->push_back(ap);
					ID++;
					//��Ϣ���������
					//	ap->GetWifiPhy()->GetwifiBandwidthManager()->Print();
					std::cout << "Created AP, id " << ap->GetIDNetworkNode()
						<< ", position: " << ap->Getposition()->GetPositionX()
						<< ", " << ap->Getposition()->GetPositionY() 
						<< ", " << ap->Getposition()->GetPositionZ() << endl;

				}
			 }

			std::cout << "�����û�" << endl;
			ID = 0;
			double  UE_X, UE_Y, UE_Z;
            //���ڷֲ���10���û�����ֲ���ÿһ���¥��,��240��
			  for (int i = 0; i < 4; i++)         
			  {
				  double x = center_pos[i][0];
				  double y = center_pos[i][1];              //������¥
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
     
			//����ֲ�������ֲ�60�ˣ��ֲ����������ɵ���
			  for (int i = 0; i < nb_totalUe - 4 * 6 * Ue_Perfloor; i++)
			  {
				  Mobility * Mob = new Mobility(SCENARIO_type, m_Outdoor, 2);   //ʵ���ƶ�

				  if (i < (nb_totalUe - 4 * 6 * Ue_Perfloor) / 2)
				  {
				    UE_X = ((double)rand() / 0x7fff * 2 - 1) * 138;   //x��-138��138��       
				    UE_Y = ((double)rand() / 0x7fff * 2 - 1) * 9 ;    // y��-9��9�� 
					UE_Z = 1.5;
				    Mob->m_speedDirection = ((double)rand() / 0x7fff * 2 - 1)>0 ? 0 : 180;    // ����������û��ƶ�����Ϊ���������� (���ȡ)
					if (UE_X>69)
						Mob->m_speedDirection = 180;
					else if (UE_X<-69)
						Mob->m_speedDirection = 0;                   //��֤�����ڼ䲻���߽�
				  }
				   else
				  {
					 double  UE_X = ((double)rand() / 0x7fff * 2 - 1) * 9;     // x��-9��9��
					 double  UE_Y = ((double)rand() / 0x7fff * 2 - 1) * 138;   //y��-138��138��
					 UE_Z = 1.5;
					 Mob->m_speedDirection = ((double)rand() / 0x7fff * 2 - 1)>0 ? 90 : -90;    // �ϱ�������û��ƶ�����Ϊ���ϻ����� (���ȡ)
					 if (UE_Y>69)
						 Mob->m_speedDirection = -90;
					 else if (UE_Y<-69)
						 Mob->m_speedDirection = 90;                   //��֤�����ڼ䲻���߽�
				  }

				  UserEquipment* user = new UserEquipment(ID, new Position(UE_X, UE_Y, UE_Z), 0, NULL, NULL, NULL,Mob);
				  user->SetPostate(m_Outdoor);
				  nm->GetUserEquipmentContainer()->push_back(user);
				  ID++;
			  }

			  //���´˴��趨�û�ҵ���Լ�ͨ��ģʽ��D2Dģʽ���߷���ģʽ��
			  for (int i = 0; i < nb_totalUe; i++)
			  {
				  UserEquipment* user = nm->GetUserEquipmentContainer()->at(i);		 
				  Application *  App_1 = new Application(Application::ApplicationType::TYPE_FULLBUFFER, 0, Sim_stop_time); //�û�����ҵ��
				  user->GetApplist()->push_back(App_1);       //����ҵ��   ���2��ҵ��       
				  user->NetSelection();					    //ҵ��ѡ�� LTE����WIFI ,   ����ȷ�����Ͷ���
				  user->wifi_link_update();                   //�����ŵ�
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

	if (SCENARIO == 2){}         //�ڶ��ೡ��


	if (SCENARIO == 3){}         //�����ೡ��


		
}






