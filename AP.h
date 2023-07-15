
#ifndef AP_H_
#define AP_H_

#include "NetworkNode.h"
#include "parameter.h"

class UserEquipment;
class APWifiPhy;
class ApMac;
class PacketBurst;
class MacQueue;
class Message;
class WIFISendingMessage;
class  CMatrix;           

using namespace std;

class AP : public NetworkNode {
public:
	struct UserEquipmentRecord
	{
		UserEquipmentRecord();                    

		virtual ~UserEquipmentRecord();
		UserEquipmentRecord(UserEquipment *UE);

		UserEquipment *m_UE;                           //用户
		vector<vector<int> > m_cqiFeedback;            //CQI反馈
		vector<vector<int> > m_PMI;                    //PMI
        int m_Mcs;                                     //调制编码
		std::vector<MacQueue*> * m_macqueue;           //对应用户的缓存队列(一个业务对应一个缓存队列)

		void SetUE(UserEquipment *UE);
		UserEquipment* GetUE() const;
	
		void SetCQI(vector<vector<int> > cqi);
		vector<vector<int> > GetCQI() const;

		void SetPMI(vector<vector<int> > pmi);
		vector<vector<int> > GetPMI() const;

		void SetMcs(int mcs);                      
		int GetMcs();

		void SetQueue(std::vector<MacQueue*> *macqueue);    //AP针对每个用户的缓存队列
		std::vector<MacQueue*> * GetQueue();


	};
    typedef std::vector<UserEquipmentRecord*> UserEquipmentRecords;

	AP();
	AP(int idElement, int Index_sector,  Position *pos);            //生成wifi AP
	virtual ~AP();
	void SetIndex_sector(int id_sector);
	int GetIndex_sector();

	void RegisterUserEquipment(UserEquipment *UE);
	void DeleteUserEquipment(UserEquipment *UE);
	
	void CreateUserEquipmentRecords();                             //用户记录表
	void DeleteUserEquipmentRecords();

	UserEquipmentRecords* GetUserEquipmentRecords();
	void SetUserEquipmentRecords(UserEquipmentRecords* v);
	UserEquipmentRecord* GetUserEquipmentRecord(int idUE);



	/*确定WIFI中资源分配问题*/

	vector<UserEquipment*>* UESieving();  //进行用户筛选

	void Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs);         //下行用户调度和资源分配

	vector<PacketBurst*>*  PacketSchedule(vector<UserEquipment*>* m_UEs);       //生成发送数据包 

	void Precoding_Generate(vector<UserEquipment*>* m_UEs);                 //生成预编码


	void SetWifiPhy(APWifiPhy *phy);                              //wifi物理层
	APWifiPhy* GetWifiPhy();



	void SetApMac(ApMac *mac);                            //mac层
	ApMac* GetApMac();


	void SendPacketBurst();                               //发包
	void ReceivePacketBurst(WIFISendingMessage* s);       //收包形式以接收用户数据信息进行


  
private:
	int Index_sector;
	APWifiPhy   *m_apwifiphy;   
	ApMac * m_apmac;

	UserEquipmentRecords *m_userEquipmentRecords;           //信息记录
};

#endif
