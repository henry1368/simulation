

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

enum UeType                    //�û�����
{
	D2D_Tx,                    //D2D�����
	D2D_Rx,                    //D2D���ջ�
	LTE_Rx,                    //�����û�
};

class UserEquipment :public NetworkNode {
public:

//	std::vector<double> Dis_UE_Enb_table, Dis_UE_Henb_table, Dis_UE_AP_table; //����
	int ID_Enb[nb_totalEnb+1]; int ID_Henb[nb_totalHenb]; int ID_AP[nb_apingroup];  //����

	struct UserEquipmentRecord
	{
		UserEquipmentRecord();                             
		virtual ~UserEquipmentRecord();

		vector<vector<int> > m_cqiFeedback;               //CQI����
		vector<vector<int> > m_PMI;                       //PMI

		int m_Mcs;                                        //���Ʊ��뷽ʽ
		vector<MacQueue*> * m_macqueue;
	};
	UserEquipmentRecord * D2D_Tx_record;

	void RegisterUserEquipment(UserEquipment *UE);                //ע���û�
	void DeleteUserEquipment(UserEquipment *UE);                  //ɾ���û�

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

	void SetIndex_sector(int id_sector);   //������
	int GetIndex_sector();
    
	void SetUEType(UeType m);                //�û�����
	UeType GetUEType();

	void SetMobilityModel(Mobility* m);    //�ƶ�ģ��
	Mobility* GetMobilityModel();

	void SetTargetAPNode(AP* n);           //����AP
	AP*  GetTargetAPNode();

	void SetTargetEnbNode(ENodeB *n);      //�����վ
	ENodeB* GetTargetEnbNode();

	void SetTargetHenbNode(HeNodeB *n);   //����С��վ
	HeNodeB* GetTargetHenbNode();

	void SetTargetUe(UserEquipment* n);   //�����û�
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
	void SetLteChannel(LteChannel* Pot);      //�û���Ӧ��Lte���ŵ�����

	WifiChannel  * GetWifiChannel();            //�û���Ӧ��Wifi���ŵ�����
	void SetWifiChannel(WifiChannel * Pot);

	vector<Application*>* GetApplist();
	void  SetApplist(vector<Application*> *m_applist );     //�û�ҵ��


	void wifi_link_update();               //1.����LTEϵͳ�е������Ϣ      ����λ�ã����¼�����·�����Լ�������
	void lte_link_update();                //2.����WIFIϵͳ�е������Ϣ      ����λ�ã����¼�����·�����Լ�������


	void Lte_SendPacketBurst(Message* s);                        //lte���з�����ʱ����
	void Wifi_SendPacketBurst(WIFISendingMessage * s);           //wifi���з�������Ϣ��ʽ����
            
	void Lte_ReceivePacketBurst(PacketBurst* p);                //lte�����հ�
	void Wifi_ReceivePacketBurst(PacketBurst* p);               //wifi�����հ�

	void D2D_SendPacketBurst(PacketBurst* p);                  //D2D����
	void D2D_ReceivePacketBurst(PacketBurst* p);               //D2D�հ�

	void NetSelection();                                      //����ѡ��



	double LTE_Receive_Data[Sim_stop_time / 1000];       //����ÿһ�ν��յ�lte���ݣ���1msΪ���������ͬʱ����յ������������˼������ͳ��
	double WIFI_Receive_Data[Sim_stop_time / 1000];      //����ÿһ�ν��յ�wifi���ݣ���1msΪ���������ͬʱ����յ������������˼������ͳ��

	void Initial_Receive_Data();



private:
	int Index_sector;

	UeType m_uetype;                         //�ڵ�����

	State m_state;                           //1Ϊ���0Ϊ����
	Mobility *m_mobility;                    //�ƶ���

	AP* m_targetAP;                          //����AP
	ENodeB * m_targetEnbNode;                //�����վ
	HeNodeB* m_targetHeNbNode;               //����С��վ
	UserEquipment* m_targetUe;               //�����û���D2D��

	UeWifiPhy *m_uewifiphy;                  //wifi�����
	UeLtePhy* m_ueltephy;                    //lte�����
	LteMac *m_ltemac;
	UeMac *m_uemac;

	LteChannel  * m_lteChannel;                //�ŵ���Ϣ
	WifiChannel  * m_wifiChannel;              
	vector<Application*>* m_Application_list;  //�û���ҵ������

};

#endif 
