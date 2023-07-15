

#ifndef USEREQUIPMENT_H_
#define USEREQUIPMENT_H_
typedef  int State;
#include <vector>
#include <cmath>
#include "ENodeB.h"
#include "HeNodeB.h"
#include "AP.h"
#include "Mobility.h"
#include "ue-wifi-phy.h"   
#include "ue-lte-phy.h"
#include "ue-mac.h"

#include "application.h"
#include "messages.h"

using namespace std;

class PacketBurst;
class LteChannel;
class WifiChannel;

enum UeType                    //用户类型
{
	D2D_Tx,                    //D2D发射机
	D2D_Rx,                    //D2D接收机
	LTE_Rx,                    //蜂窝用户
};

class UserEquipment :public NetworkNode {
public:

//	std::vector<double> Dis_UE_Enb_table, Dis_UE_Henb_table, Dis_UE_AP_table; //距离
	int ID_Enb[nb_totalEnb+1]; int ID_Henb[nb_totalHenb]; int ID_AP[nb_apingroup];  //排序

	struct UserEquipmentRecord
	{
		UserEquipmentRecord();                             
		virtual ~UserEquipmentRecord();

		vector<vector<int> > m_cqiFeedback;               //CQI反馈
		vector<vector<int> > m_PMI;                       //PMI

		int m_Mcs;                                        //调制编码方式
		vector<MacQueue*> * m_macqueue;
	};
	UserEquipmentRecord * D2D_Tx_record;

	void RegisterUserEquipment(UserEquipment *UE);                //注册用户
	void DeleteUserEquipment(UserEquipment *UE);                  //删除用户

	UserEquipment ();

	UserEquipment(int idElement,
		Position* position,
		int id_sector,
		ENodeB * targetEnbNode,
		HeNodeB * targetHenbNode,
		AP * targetAPNode,
	    Mobility *mobility
		);

	virtual ~UserEquipment();

	void SetIndex_sector(int id_sector);   //扇区号
	int GetIndex_sector();
    
	void SetUEType(UeType m);                //用户类型
	UeType GetUEType();

	void SetMobilityModel(Mobility* m);    //移动模型
	Mobility* GetMobilityModel();

	void SetTargetAPNode(AP* n);           //接入AP
	AP*  GetTargetAPNode();

	void SetTargetEnbNode(ENodeB *n);      //接入基站
	ENodeB* GetTargetEnbNode();

	void SetTargetHenbNode(HeNodeB *n);   //接入小基站
	HeNodeB* GetTargetHenbNode();

	void SetTargetUe(UserEquipment* n);   //接入用户
	UserEquipment*  GetTargetUe();

	void SetWifiPhy(UeWifiPhy *phy);      //UeWifiPhy
	UeWifiPhy* GetWifiPhy();
	
	void SetLtePhy(UeLtePhy *phy);       //UeLtePhy
	UeLtePhy* GetLtePhy();

	void SetLteMac(LteMac *mac);         //LteMac
	LteMac* GetLteMac();

	void SetUeMac(UeMac *mac);           //UeMac
	UeMac* GetUeMac();

	LteChannel * GetLteChannel();
	void SetLteChannel(LteChannel* Pot);      //用户对应给Lte的信道矩阵

	WifiChannel  * GetWifiChannel();            //用户对应给Wifi的信道矩阵
	void SetWifiChannel(WifiChannel * Pot);

	vector<Application*>* GetApplist();
	void  SetApplist(vector<Application*> *m_applist );     //用户业务


	void wifi_link_update();               //1.更新LTE系统中的相关信息      更新位置，重新计算链路距离以及可视性
	void lte_link_update();                //2.更新WIFI系统中的相关信息      更新位置，重新计算链路距离以及可视性


	void Lte_SendPacketBurst(Message* s);                        //lte上行发包暂时不仿
	void Wifi_SendPacketBurst(WIFISendingMessage * s);           //wifi上行发包以信息形式发送
            
	void Lte_ReceivePacketBurst(PacketBurst* p);                //lte下行收包
	void Wifi_ReceivePacketBurst(PacketBurst* p);               //wifi下行收包

	void D2D_SendPacketBurst(PacketBurst* p);                  //D2D发包
	void D2D_ReceivePacketBurst(PacketBurst* p);               //D2D收包

	void NetSelection();                                      //网络选择



	double LTE_Receive_Data[Sim_stop_time / 1000];       //储存每一次接收的lte数据，以1ms为间隔，将不同时间接收的数据量化到此间隔进型统计
	double WIFI_Receive_Data[Sim_stop_time / 1000];      //储存每一次接收的wifi数据，以1ms为间隔，将不同时间接收的数据量化到此间隔进型统计

	void Initial_Receive_Data();



private:
	int Index_sector;

	UeType m_uetype;                         //节点类型

	State m_state;                           //1为激活，0为休眠
	Mobility *m_mobility;                    //移动性

	AP* m_targetAP;                          //接入AP
	ENodeB * m_targetEnbNode;                //接入基站
	HeNodeB* m_targetHeNbNode;               //接入小基站
	UserEquipment* m_targetUe;               //连接用户（D2D）

	UeWifiPhy *m_uewifiphy;                  //wifi物理层
	UeLtePhy* m_ueltephy;                    //lte物理层
	LteMac *m_ltemac;
	UeMac *m_uemac;

	LteChannel  * m_lteChannel;                //信道信息
	WifiChannel  * m_wifiChannel;              
	vector<Application*>* m_Application_list;  //用户的业务容器

};

#endif 
