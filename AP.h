
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

		UserEquipment *m_UE;                           //�û�
		vector<vector<int> > m_cqiFeedback;            //CQI����
		vector<vector<int> > m_PMI;                    //PMI
        int m_Mcs;                                     //���Ʊ���
		std::vector<MacQueue*> * m_macqueue;           //��Ӧ�û��Ļ������(һ��ҵ���Ӧһ���������)

		void SetUE(UserEquipment *UE);
		UserEquipment* GetUE() const;
	
		void SetCQI(vector<vector<int> > cqi);
		vector<vector<int> > GetCQI() const;

		void SetPMI(vector<vector<int> > pmi);
		vector<vector<int> > GetPMI() const;

		void SetMcs(int mcs);                      
		int GetMcs();

		void SetQueue(std::vector<MacQueue*> *macqueue);    //AP���ÿ���û��Ļ������
		std::vector<MacQueue*> * GetQueue();


	};
    typedef std::vector<UserEquipmentRecord*> UserEquipmentRecords;

	AP();
	AP(int idElement, int Index_sector,  Position *pos);            //����wifi AP
	virtual ~AP();
	void SetIndex_sector(int id_sector);
	int GetIndex_sector();

	void RegisterUserEquipment(UserEquipment *UE);
	void DeleteUserEquipment(UserEquipment *UE);
	
	void CreateUserEquipmentRecords();                             //�û���¼��
	void DeleteUserEquipmentRecords();

	UserEquipmentRecords* GetUserEquipmentRecords();
	void SetUserEquipmentRecords(UserEquipmentRecords* v);
	UserEquipmentRecord* GetUserEquipmentRecord(int idUE);



	/*ȷ��WIFI����Դ��������*/

	vector<UserEquipment*>* UESieving();  //�����û�ɸѡ

	void Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs);         //�����û����Ⱥ���Դ����

	vector<PacketBurst*>*  PacketSchedule(vector<UserEquipment*>* m_UEs);       //���ɷ������ݰ� 

	void Precoding_Generate(vector<UserEquipment*>* m_UEs);                 //����Ԥ����


	void SetWifiPhy(APWifiPhy *phy);                              //wifi�����
	APWifiPhy* GetWifiPhy();



	void SetApMac(ApMac *mac);                            //mac��
	ApMac* GetApMac();


	void SendPacketBurst();                               //����
	void ReceivePacketBurst(WIFISendingMessage* s);       //�հ���ʽ�Խ����û�������Ϣ����


  
private:
	int Index_sector;
	APWifiPhy   *m_apwifiphy;   
	ApMac * m_apmac;

	UserEquipmentRecords *m_userEquipmentRecords;           //��Ϣ��¼
};

#endif
