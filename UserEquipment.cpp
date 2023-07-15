
#include "UserEquipment.h"
#include "NetworkNode.h"
#include "ENodeB.h"
#include "AP.h"
#include "wifi-bandwidth-manager.h"
#include "lte-bandwidth-manager.h"
#include "wifi-phy.h"
#include "ue-wifi-phy.h"
#include "ap-wifi-phy.h"
#include "LTE-A_channel.h"
#include "WIFI_channel.h"
#include "lte-propagation-loss-model.h"
#include "wifi-propagation-loss-model.h"
#include  "MadridGrid-propagation-model.h"
#include  "urban-micro-propagation-model.h"
#include "lte-phy.h"
#include "ue-lte-phy.h"
#include "henb-lte-phy.h"
#include "enb-lte-phy.h"
#include "ue-mac.h"
#include "Mobility.h"
#include "position.h"
#include "MacQueue.h"
#include "NetworkManager.h"
#include "parameter.h"
#include "math.h"
#include "application.h"
#include "simulator.h"
#include  "application.h"
#include "lte-mac.h"

template < typename T1>
void swap(T1 array[], int i, int j);

void SelectionSort(double array[], int array1[], int n);


UserEquipment::UserEquipment ()
{}

UserEquipment::UserEquipment (int idElement,
	                          Position* position,
							  int id_sector,
							  ENodeB * targetEnbNode,
							  HeNodeB * targetHenbNode,
							  AP * targetAPNode,
							  Mobility *mobility
							  )
{

  SetNodeType(TYPE_UE);                 //设定类型
  SetIDNetworkNode (idElement);
  Setposition(position);
  m_state=1;
  SetIndex_sector(id_sector);
  SetMobilityModel(mobility);


  
  SetTargetEnbNode(targetEnbNode);    //物理层也在此时生成
  SetTargetHenbNode(targetHenbNode);  //物理层也在此时生成
  SetTargetAPNode(targetAPNode);

  SetTargetUe(NULL);
  D2D_Tx_record = NULL;         //默认以蜂窝用户生成
  SetUEType(LTE_Rx);


  UeLtePhy *phy = new UeLtePhy();  //lte物理层
  phy->SetDevice(this);
  SetLtePhy(phy);

  UeWifiPhy *wphy = new UeWifiPhy();  //wifi物理层
  wphy->SetDevice(this);
  SetWifiPhy(wphy);

  UeMac *mac = new UeMac();          //mac层
  mac->SetDevice(this);
  SetUeMac(mac);

  LteMac *ltemac = new LteMac();  //lte mac
  ltemac->SetDevice(this);
  SetLteMac(ltemac);

  LteChannel* ltechannel = new LteChannel();  //初始化LTE信道
  SetLteChannel(ltechannel);
  ltechannel->SetUe(this);


  WifiChannel* wifichannel = new WifiChannel();//初始化WIFI信道
  SetWifiChannel(wifichannel);
  wifichannel->SetUe(this);

  m_Application_list = new vector<Application*>;   //初始化用户业务列表

  Initial_Receive_Data();

}

UserEquipment::~UserEquipment()
{
	m_targetEnbNode = NULL;
	m_targetAP = NULL;                   
	m_mobility = NULL;
	m_targetUe = NULL;
	delete  m_ueltephy;
	delete  m_uewifiphy;
	delete  m_uemac;
    Destroy ();

}


void UserEquipment::lte_link_update()          //重新计算用户到基站距离，确定连接目标，在此之前应该是更新位置
{
	if (SCENARIO_type == "TC1")
	{
	Position* pos = Getposition();                    //用户位置
	GetMobilityModel()->UpdatePosition(pos);          //用户更新位置

//判断是否发生位置显著变化

	Position* last_pos= GetMobilityModel()->m_last_update_lte_position;   //上次计算大尺度的位置:m_last_update_lte_position在SetMobilityModel(Mobility* m)函数中已经初始化
	double m_distance = distance(last_pos, pos);                          //变化距离

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)     //位置距离发生显著变化，重新计算用户到所有基站的距离及LOS参数以及信道系数（大尺度、小尺度）等, 并判断是否需要切换
	{
		GetMobilityModel()->m_last_update_lte_position = pos;

		NetworkManager* nm = NetworkManager::Init();
	//	std::vector<ENodeB*>* m_ENodeBContainer = nm->GetENodeBContainer();
		std::vector<HeNodeB*>* m_HeNodeBContainer = nm->GetHeNodeBContainer();

		if (GetUEType() == LTE_Rx)
		{
		//	double best_enb_loss = -100000; int id_target_enb = -1;                     //距离和连接全部都重新初始化
			double best_henb_loss = -100000; int id_target_henb = -1;

		//    GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_ENodeBContainer);
			GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_HeNodeBContainer);
			//用户到每个宏基站的信道大尺度进行排序，确定信号最强的前几个作为干扰处理  
			/*
			for (int i = 0; i < nb_totalEnb; i++)
			{
				double temp_eNodeB_loss[nb_totalEnb + 1] = { 0.0 };                 //+1是因为C++不允许定义大小为0的数组
				for (int i = 0; i < nb_totalEnb; i++)                               //将信道衰落矩阵赋值给临时矩阵
				{
					temp_eNodeB_loss[i] = GetLteChannel()->eNodeB_power_loss[i];
					ID_Enb[i] = i;
				}
				SelectionSort(temp_eNodeB_loss, ID_Enb, nb_totalEnb);              //排序，信道衰落从好到差，对应基站ID也变换
				best_enb_loss = temp_eNodeB_loss[0];
				id_target_enb = ID_Enb[0];
			}
			*/
			//更新用户到每个小基站的信道大尺度进行排序，确定信号最强的前几个作为干扰处理  
			for (int i = 0; i < nb_totalHenb; i++)
			{
				double temp_HeNodeB_loss[nb_totalHenb] = { 0.0 };
				for (int i = 0; i < nb_totalHenb; i++)
				{
					temp_HeNodeB_loss[i] = GetLteChannel()->HeNodeB_power_loss[i];
					ID_Henb[i] = i;
				}
				SelectionSort(temp_HeNodeB_loss, ID_Henb, nb_totalHenb);
				best_henb_loss = temp_HeNodeB_loss[0];
				id_target_henb = ID_Henb[0];
			}

			//确定连接对象（比较宏基站和小基站信道条件最好的情况）
			//选择连接宏基站,当先前所连的基站类型为小基站，或为宏基站但编号与当前不一致时需要重新注册和删除先前记录
			
				if (GetTargetHenbNode() == NULL || GetTargetHenbNode()->GetIDNetworkNode() != id_target_henb)
				{
					SetTargetHenbNode(m_HeNodeBContainer->at(id_target_henb));
					if (GetTargetEnbNode() != NULL)
						GetTargetEnbNode()->DeleteUserEquipment(this);
				}
			
           /*
			vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
			for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //统计出信号最强的Num_Enb_Select+1的宏基站
			{
				int N = ID_Enb[KK];                             //宏基站编号
				m_ENodeBs->push_back(m_ENodeBContainer->at(N));
			}
			*/
			vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
			for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //统计出信号最强的Num_Henb_Select+1的小基站
			{
				int N = ID_Henb[KK];                             //小基站编号
				m_HeNodeBs->push_back(m_HeNodeBContainer->at(N));
			}

		

			
		//	GetLteChannel()->GetLtePropagationLossModel()->calculateNodeB_medi(this, m_ENodeBs);                            //重新计算小尺度中间参数
			GetLteChannel()->GetLtePropagationLossModel()->calculateHeNodeB_medi(this, m_HeNodeBs);                         //重新计算小尺度中间参数

		

		//	GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));  //重新计算小尺度参数

		//	delete m_ENodeBs;
			delete m_HeNodeBs;

       }
		else
		{
			//非蜂窝用户
		}		
	}

      else
	{		
	    //未发生明显距离变化，则大尺度无需重新计算
		/*
	     vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
	     for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //统计出信号最强的Num_Enb_Select+1的宏基站
	   {
			int N = ID_Enb[KK];                             //宏基站编号
			m_ENodeBs->push_back(NetworkManager::Init()->GetENodeBContainer()->at(N));
	    }

		  vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
		 for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //统计出信号最强的Num_Henb_Select+1的小基站
	   {
			int N = ID_Henb[KK];                             //小基站编号
			m_HeNodeBs->push_back(NetworkManager::Init()->GetHeNodeBContainer()->at(N));
	    }
	
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));
		
		delete m_ENodeBs;
		delete m_HeNodeBs;
		*/
	}
	}
	else
	{
	Position* pos = Getposition();                    //用户位置
	GetMobilityModel()->UpdatePosition(pos);          //用户更新位置

//判断是否发生位置显著变化

	Position* last_pos= GetMobilityModel()->m_last_update_lte_position;   //上次计算大尺度的位置:m_last_update_lte_position在SetMobilityModel(Mobility* m)函数中已经初始化
	double m_distance = distance(last_pos, pos);                          //变化距离

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)     //位置距离发生显著变化，重新计算用户到所有基站的距离及LOS参数以及信道系数（大尺度、小尺度）等, 并判断是否需要切换
	{
		GetMobilityModel()->m_last_update_lte_position = pos;

		NetworkManager* nm = NetworkManager::Init();
		std::vector<ENodeB*>* m_ENodeBContainer = nm->GetENodeBContainer();
		std::vector<HeNodeB*>* m_HeNodeBContainer = nm->GetHeNodeBContainer();

		if (GetUEType() == LTE_Rx)
		{
			double best_enb_loss = -100000; int id_target_enb = -1;                     //距离和连接全部都重新初始化
			double best_henb_loss = -100000; int id_target_henb = -1;

		    GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_ENodeBContainer);
			GetLteChannel()->GetLtePropagationLossModel()->Compute_PowerLoss(this, m_HeNodeBContainer);
			//用户到每个宏基站的信道大尺度进行排序，确定信号最强的前几个作为干扰处理    
			for (int i = 0; i < nb_totalEnb; i++)
			{
				double temp_eNodeB_loss[nb_totalEnb + 1] = { 0.0 };                 //+1是因为C++不允许定义大小为0的数组
				for (int i = 0; i < nb_totalEnb; i++)                               //将信道衰落矩阵赋值给临时矩阵
				{
					temp_eNodeB_loss[i] = GetLteChannel()->eNodeB_power_loss[i];
					ID_Enb[i] = i;
				}
				SelectionSort(temp_eNodeB_loss, ID_Enb, nb_totalEnb);              //排序，信道衰落从好到差，对应基站ID也变换
				best_enb_loss = temp_eNodeB_loss[0];
				id_target_enb = ID_Enb[0];
			}

			//更新用户到每个小基站的信道大尺度进行排序，确定信号最强的前几个作为干扰处理  
			for (int i = 0; i < nb_totalHenb; i++)
			{
				double temp_HeNodeB_loss[nb_totalHenb] = { 0.0 };
				for (int i = 0; i < nb_totalHenb; i++)
				{
					temp_HeNodeB_loss[i] = GetLteChannel()->HeNodeB_power_loss[i];
					ID_Henb[i] = i;
				}
				SelectionSort(temp_HeNodeB_loss, ID_Henb, nb_totalHenb);
				best_henb_loss = temp_HeNodeB_loss[0];
				id_target_henb = ID_Henb[0];
			}

			//确定连接对象（比较宏基站和小基站信道条件最好的情况）
			//选择连接宏基站,当先前所连的基站类型为小基站，或为宏基站但编号与当前不一致时需要重新注册和删除先前记录
			if (best_enb_loss >= best_henb_loss)
			{
				if (GetTargetEnbNode() == NULL || GetTargetEnbNode()->GetIDNetworkNode() != id_target_enb)
				{
					SetTargetEnbNode(m_ENodeBContainer->at(id_target_enb));
					if (GetTargetHenbNode() != NULL)                                //删除记录
						GetTargetHenbNode()->DeleteUserEquipment(this);
				}
			}
			//选择连接小基站,当先前所连的基站类型为宏基站，或为小基站但编号与当前不一致时需要重新注册和删除先前记录
			else
			{
				if (GetTargetHenbNode() == NULL || GetTargetHenbNode()->GetIDNetworkNode() != id_target_henb)
				{
					SetTargetHenbNode(m_HeNodeBContainer->at(id_target_henb));
					if (GetTargetEnbNode() != NULL)
						GetTargetEnbNode()->DeleteUserEquipment(this);
				}
			}

			vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
			for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //统计出信号最强的Num_Enb_Select+1的宏基站
			{
				int N = ID_Enb[KK];                             //宏基站编号
				m_ENodeBs->push_back(m_ENodeBContainer->at(N));
			}

			vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
			for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //统计出信号最强的Num_Henb_Select+1的小基站
			{
				int N = ID_Henb[KK];                             //小基站编号
				m_HeNodeBs->push_back(m_HeNodeBContainer->at(N));
			}

		

			
			GetLteChannel()->GetLtePropagationLossModel()->calculateNodeB_medi(this, m_ENodeBs);                            //重新计算小尺度中间参数
			GetLteChannel()->GetLtePropagationLossModel()->calculateHeNodeB_medi(this, m_HeNodeBs);                         //重新计算小尺度中间参数

		

			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));  //重新计算小尺度参数

			delete m_ENodeBs;
			delete m_HeNodeBs;

       }
		else
		{
			//非蜂窝用户
		}		
	}

      else
	{		
	    //未发生明显距离变化，则大尺度无需重新计算
		/*
	     vector<ENodeB*>* m_ENodeBs = new vector<ENodeB*>;
	     for (int KK = 0; KK < min(Num_Enb_Select + 1, nb_totalEnb); KK++)    //统计出信号最强的Num_Enb_Select+1的宏基站
	   {
			int N = ID_Enb[KK];                             //宏基站编号
			m_ENodeBs->push_back(NetworkManager::Init()->GetENodeBContainer()->at(N));
	    }

		  vector<HeNodeB*>* m_HeNodeBs = new vector<HeNodeB*>;
		 for (int KK = 0; KK < min(Num_Henb_Select + 1, nb_totalHenb); KK++)    //统计出信号最强的Num_Henb_Select+1的小基站
	   {
			int N = ID_Henb[KK];                             //小基站编号
			m_HeNodeBs->push_back(NetworkManager::Init()->GetHeNodeBContainer()->at(N));
	    }
	
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_ENodeBs, (Simulator::Init()->Now() / 1000));
			GetLteChannel()->GetLtePropagationLossModel()->Compute_SmallscaleFading(this, m_HeNodeBs, (Simulator::Init()->Now() / 1000));
		
		delete m_ENodeBs;
		delete m_HeNodeBs;
		*/
	}
	
	}	
}




void UserEquipment::wifi_link_update()      //重新计算用户到AP距离矩阵，确定连接目标，在此之前应该是更新位置
{

	Position* pos = Getposition();            //当前用户位置
	GetMobilityModel()->UpdatePosition(pos);  //用户更新位置

	Position* last_pos = GetMobilityModel()->m_last_update_wifi_position; //上次计算大尺度的位置：m_last_update_wifi_position 在SetMobilityModel(Mobility* m)函数中已经初始化
	double m_distance = distance(last_pos, pos);                          //变化距离

	if (sqrt(m_distance) > 30 || Simulator::Init()->Now() == 0.0)          //位置距离发生显著变化，则进行大尺度和小尺度的重新计算
	{

		GetMobilityModel()->m_last_update_wifi_position = pos;	
		vector<AP*>* m_APContainer = NetworkManager::Init()->GetAPContainer();
		vector<AP*>* m_APs = NULL;

		if (strcmp(SCENARIO_type, "TC2") == 0)      //对于TC2场景，室外用户只考虑与大楼第一层的wifi AP相连，室内用户只与该层的wifi AP相连
		{
			m_APs = new vector<AP*>;
			vector<AP*>::iterator iter;
			for (iter = m_APContainer->begin(); iter != m_APContainer->end(); iter++)
			{
				double z1 = (*iter)->Getposition()->GetPositionZ();      //AP的Z坐标
				double z2 = Getposition()->GetPositionZ();
				double z3 = max(z1 - z2, z2 - z1);
				if (z3 < 3.5)                          //垂直距离在同一楼层范围内  					 
					m_APs->push_back((*iter));
			}
		}
		else
			m_APs = m_APContainer;              //其它场景全部计算
		
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_PowerLoss(this, m_APs);

		double best_AP_loss; int id_target_AP;
		double *temp_AP_loss = new double[m_APs->size()];

		for (unsigned int i = 0; i < m_APs->size(); i++)
		{
			ID_AP[i] = m_APs->at(i)->GetIDNetworkNode();
			temp_AP_loss[i] = GetWifiChannel()->AP_power_loss[ID_AP[i]];
		}
		SelectionSort(temp_AP_loss, ID_AP, m_APs->size());
		best_AP_loss = temp_AP_loss[0];
		id_target_AP = ID_AP[0];

		if (this->GetTargetAPNode() != m_APContainer->at(id_target_AP))
			SetTargetAPNode(m_APContainer->at(id_target_AP));                         //选择连接信号最强的AP


		//只考虑同群干扰，因此只计算到该群的所有小尺度参数
		int k = 1;
		for (int i = (id_target_AP / nb_apingroup)* nb_apingroup; i < (id_target_AP / nb_apingroup + 1)* nb_apingroup; i++)  //确定和所连AP的同群的其他AP
		{		          
			if (id_target_AP != i)
			{			
				ID_AP[k] = i;                   //ID_AP
			    k++;
			}
		}

		vector<AP*>* temp_APs = new vector<AP*>;
		for (int i = 0; i < nb_apingroup; i++)
		{
			int id = ID_AP[i];
			temp_APs->push_back(m_APContainer->at(id));			
		}
	    

		GetWifiChannel()->GetWifiPropagationLossModel()->calculateAP_medi(this, temp_APs);	     //重新计算大尺度，以及小尺度的中间变量
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_SmallscaleFading(this, temp_APs, (Simulator::Init()->Now() / 1000));

		delete temp_APs;
	}
	else
	{
      //未发生明显距离变化，则大尺度无需重新计算
		/*
		vector<AP*>* temp_APs = new vector<AP*>;
		for (int i = 0; i < nb_apingroup; i++)
		{
			int id = ID_AP[i];
			temp_APs->push_back(NetworkManager::Init()->GetAPContainer()->at(id));
		}

		GetWifiChannel()->GetWifiPropagationLossModel()->calculateAP_medi(this, temp_APs);	     //重新计算大尺度，以及小尺度的中间变量
		GetWifiChannel()->GetWifiPropagationLossModel()->Compute_SmallscaleFading(this, temp_APs, (Simulator::Init()->Now() / 1000));

		delete temp_APs;
		*/
	}
	
}





void 
UserEquipment::SetIndex_sector(int id_sector)
{
	Index_sector = id_sector;
}

int 
UserEquipment::GetIndex_sector()
{
	return Index_sector;
}


void 
UserEquipment::SetUEType(UeType m)               //用户类型
{
	m_uetype = m;
	if (m != LTE_Rx)
	   GetLtePhy()->SetlteBandwidthManager(new lte_BandwidthManager("enb"));    //D2D链路复用蜂窝下行频率资源
}


UeType 
UserEquipment::GetUEType()
{
	return m_uetype;
}


Mobility*
UserEquipment::GetMobilityModel()             //移动性
{
	return m_mobility;
}


void
UserEquipment::SetMobilityModel(Mobility* m)
{
	m->m_last_update_lte_position = Getposition();
	m->m_last_update_wifi_position = Getposition();   
	m_mobility = m;
}


void UserEquipment::SetTargetEnbNode(ENodeB *n)         //链接基站
{
	m_targetEnbNode = n;
    if (n != NULL)
	{
	GetLtePhy()->SetlteBandwidthManager(n->GetLtePhy()->GetlteBandwidthManager());	
	 n->RegisterUserEquipment(this);
	}
}

ENodeB *
UserEquipment::GetTargetEnbNode()
{
	return m_targetEnbNode;
}


void UserEquipment::SetTargetHenbNode(HeNodeB *n)         //关于接入小基站
{		
	m_targetHeNbNode = n;
	if (n != NULL)
	{
		GetLtePhy()->SetlteBandwidthManager(n->GetLtePhy()->GetlteBandwidthManager());
		n->RegisterUserEquipment(this);
	}
}

HeNodeB *
UserEquipment::GetTargetHenbNode()
{
	return m_targetHeNbNode;

}


void UserEquipment::SetTargetAPNode(AP* n)                  //连接wifi AP
{

	m_targetAP = n;
	if (n!=NULL)
{
	GetWifiPhy()->SetwifiBandwidthManager(n->GetWifiPhy()->GetwifiBandwidthManager());	
	n->RegisterUserEquipment(this);
}

}


AP* UserEquipment::GetTargetAPNode()
{
	return m_targetAP;
}


void UserEquipment::SetTargetUe(UserEquipment* n)   //接入用户
{
	m_targetUe = n;

	if (GetUEType() == D2D_Tx)       //发射端注册D2D_Rx的信息
	{
		RegisterUserEquipment(n);
	}
}


UserEquipment*  
UserEquipment::GetTargetUe()
{
	return m_targetUe;
}


void UserEquipment::SetWifiPhy(UeWifiPhy *phy)      //关于wifi物理层
{
	m_uewifiphy = phy;
}

UeWifiPhy* 
UserEquipment::GetWifiPhy()
{
	return m_uewifiphy;
}



void UserEquipment::SetLtePhy(UeLtePhy *phy)                     //关于Lte物理层
{
	m_ueltephy=phy;
}

UeLtePhy* 
UserEquipment::GetLtePhy()
{
	return m_ueltephy;
}

void
UserEquipment::SetLteMac(LteMac *mac)                     //关于mac层
{
	m_ltemac = mac;
}

LteMac*
UserEquipment::GetLteMac()
{
	return m_ltemac;
}

void
UserEquipment::SetUeMac(UeMac *mac)                     //关于mac层
{
	m_uemac = mac;
}

UeMac*
UserEquipment::GetUeMac()
{
	return m_uemac;
}


LteChannel * 
UserEquipment::GetLteChannel()
{
	return m_lteChannel;
}

void
UserEquipment::SetLteChannel(LteChannel * Pot)    //用户对应给Lte的信道矩阵
{
	m_lteChannel = Pot;
}

WifiChannel  *
UserEquipment::GetWifiChannel()                   //用户对应给Wifi的信道矩阵
{
	return m_wifiChannel;
}

void 
UserEquipment::SetWifiChannel(WifiChannel* Pot)
{
	m_wifiChannel = Pot;
}

vector<Application*>* 
UserEquipment::GetApplist()
{
	return m_Application_list;
}

void  
UserEquipment::SetApplist(vector<Application*> *m_applist)     //用户业务
{
	m_Application_list = m_applist;
}


void 
UserEquipment::NetSelection()                                    //支持多种业务
{

	//设定0、1、2分别为业务的选择网络类型
	/*
	0: Lte网络；
	1：Wifi网络；
	2：Lte和Wifi网络共同发送
	*/

	vector<Application*>::iterator iter;
	for (iter = m_Application_list->begin(); iter != m_Application_list->end(); iter++)
	{
		Application * app =(*iter);
		if ((app->GetApptype() == Application::ApplicationType::TYPE_VOIP) || (app->GetApptype() == Application::ApplicationType::TYPE_WEB))
		{
			     app->m_Num_of_net = 0;                    //如果是语音或网页业务，则将该业务分配给基站		
		}

		else if ((app->GetApptype() == Application::ApplicationType::TYPE_VIDEO) || (app->GetApptype() == Application::ApplicationType::TYPE_GAME))
		{
		
			     app->m_Num_of_net = 1;                   //如果该业务是视频或游戏业务，将该业务分配给WIFI网络，此时并不是直接拷给AP,而是通过用户竞争发送的形式给AP；
        }
		else if ((app->GetApptype() == Application::ApplicationType::TYPE_FTP) || (app->GetApptype() == Application::ApplicationType::TYPE_FULLBUFFER))

		{
			     app->m_Num_of_net = 2;                  //如果该业务是下载或fullbuffer业务，将该业务分配给WIFI网络和LTE网络；	

				 int M = (app->GetDatalist()->back()->m_index)+1;
				 app->m_flag_ptr = new int[M+1];
				 app->m_Fountain_data = new long double[M];
				 for (int i = 0; i < M; i++)
				 {
					 app->m_flag_ptr[i] = 0;            //设置该业务中的数据流的标志均为0，用于喷泉码中两种网络的同步；
					 app->m_Fountain_data[i] = 0;     //设置从两个网络接收到的数据包总数
				 }


		}

	}

}



void
UserEquipment::Lte_SendPacketBurst(Message* s)                                     //lte发包
{
    //上行链路不仿
}


void
UserEquipment::Wifi_SendPacketBurst(WIFISendingMessage * s)                                     //用户Wifi下给ap发包
{
	//WIFI中上行链路中用户参与信道竞争，由于仿真侧重下行链路，因此将上行链路的用户发包以消息的形式进行发送
	cout << "用户" << GetIDNetworkNode() << "号在时刻 t=" << Simulator::Init()->Now() << "发送业务需求的信息：" << endl;

	Simulator::Init()->Schedule("wifi", 3, &AP::ReceivePacketBurst, (AP*)s->m_destination, s); //将对应的接收事件入队    3us后AP收到用户的数据信息

      
} 


void UserEquipment::Lte_ReceivePacketBurst(PacketBurst* p)
{
	//收包在ue-lte-phy
}

void UserEquipment::Wifi_ReceivePacketBurst(PacketBurst* p)
{
	//收包在ue-wifi-phy
}




void D2D_SendPacketBurst(PacketBurst* p);                  //D2D发包
void D2D_ReceivePacketBurst(PacketBurst* p);               //D2D收包



void UserEquipment::Initial_Receive_Data()
{
	for (int i = 0; i < Sim_stop_time / 1000; i++)          //初始化用户接收数据为0
	{
		LTE_Receive_Data[i] = 0;
		WIFI_Receive_Data[i] = 0;  
	}
                            
}


void
UserEquipment::RegisterUserEquipment(UserEquipment *UE)
{

	D2D_Tx_record = new UserEquipmentRecord();
	D2D_Tx_record->m_cqiFeedback.clear();
	D2D_Tx_record->m_PMI.clear();

	D2D_Tx_record->m_macqueue = new vector<MacQueue*>;                              //建立队列
	for (unsigned int i = 0; i<UE->GetApplist()->size(); i++)                       //每个队列对应一个业务   //这里需要判断是否是选择的LTE网络
	{
		if (UE->GetApplist()->at(i)->m_Num_of_net == 0 || UE->GetApplist()->at(i)->m_Num_of_net == 2)  //该业务走LTE网络
		{
			MacQueue *m_queue = new MacQueue();
			m_queue->SetAppNum(i);                                            //记录业务数
			list<Application::flowNode *>* datalist = UE->GetApplist()->at(i)->GetDatalist();
			list<Application::flowNode *>::iterator it;
			for (it = datalist->begin(); it != datalist->end(); it++)        //每个业务一个队列
				m_queue->Enqueue((*it));                                //根据业务数据生成对应的队列
			D2D_Tx_record->m_macqueue->push_back(m_queue);
		}
	}

	for (int i = 0; i < FeedBackDelay + 1; i++)     //初始化用户的cqi/pmi,这里考虑反馈延时，生成延时循环
	{
		vector<int> temp_vector;
		for (int j = 0; j < RBs_FOR_LTE; j++)
		{
			temp_vector.push_back(int(rand()) % 10);
		}
		D2D_Tx_record->m_cqiFeedback.push_back(temp_vector);
		D2D_Tx_record->m_PMI.push_back(temp_vector);
		temp_vector.clear();
	}
}


void
UserEquipment::DeleteUserEquipment(UserEquipment *UE)
{
	delete D2D_Tx_record;
}



UserEquipment::UserEquipmentRecord::UserEquipmentRecord()
{
	//Create initial CQI values:
	m_cqiFeedback.clear();
	m_PMI.clear();
}

UserEquipment::UserEquipmentRecord::~UserEquipmentRecord()
{
	m_cqiFeedback.clear();
	m_PMI.clear();
}


/*交换函数，作用是交换数组中的两个元素的位置*/

template < typename T1>
void swap(T1 array[], int i, int j)
{
	T1 tmp = array[i];
	array[i] = array[j];
	array[j] = tmp;
}


/*选择排序*/
void SelectionSort(double array[], int array1[], int n)
{
	for (int i = 0; i<n - 1; i++)
	{
		int biggest = i;
		for (int j = i + 1; j<n; j++)
		{
			if (array[biggest]<array[j])
				biggest = j;
		}
		swap(array, i, biggest);
		swap(array1, i, biggest);
	}
}
