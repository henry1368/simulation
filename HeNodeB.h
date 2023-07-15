
#ifndef HENODEB_H_
#define HENODEB_H_

#include "ENodeB.h"

class UserEquipment;
class HenbLtePhy;
class LteMac;
class PacketBurst;
class MacQueue;


class HeNodeB : public NetworkNode {
public:

	struct UserEquipmentRecord
	{
		UserEquipmentRecord();        //
		virtual ~UserEquipmentRecord();
		UserEquipmentRecord(UserEquipment *UE);

		UserEquipment *m_UE;                               //�û�
		vector<vector<int> > m_cqiFeedback;                    //CQI����
		vector<vector<int> > m_PMI;                            //PMI

		int m_Mcs;                                       //���Ʊ��뷽ʽ
		vector<MacQueue*> * m_macqueue;               //��Ӧ�û��Ļ������(һ��ҵ���Ӧһ���������)

		void SetUE(UserEquipment *UE);
		UserEquipment* GetUE() const;

		void SetCQI(vector<vector<int> > cqi);
		vector<vector<int> > GetCQI() const;

		void SetPMI(vector<vector<int> > pmi);
		vector<vector<int> > GetPMI() const;

		void SetMcs(int mcs);
		int GetMcs();

		void SetQueue(vector<MacQueue*> *macqueue);
		vector<MacQueue*> * GetQueue();

	};

	typedef vector<UserEquipmentRecord*> UserEquipmentRecords;

	enum DLSchedulerType                                                            //���е��ȷ�ʽ
	{
		DLScheduler_TYPE_MAXIMUM_THROUGHPUT,
		DLScheduler_TYPE_PROPORTIONAL_FAIR,
		DLScheduler_TYPE_FLS,
		DLScheduler_TYPE_MLWDF,
		DLScheduler_TYPE_EXP,
		DLScheduler_LOG_RULE,
		DLScheduler_EXP_RULE
	};


	HeNodeB();
	HeNodeB(int idElement, int Index_sector, Position * pos);                    //����С��վ 

	virtual ~HeNodeB();

	void SetIndex_sector(int id_sector);
	int GetIndex_sector();

	void RegisterUserEquipment(UserEquipment *UE);                                 //ע���û�
	void DeleteUserEquipment(UserEquipment *UE);                                   //ɾ���û�

	void CreateUserEquipmentRecords();                                         //�û���¼��
	void DeleteUserEquipmentRecords();                                         //ɾ���û���¼��  

	UserEquipmentRecords* GetUserEquipmentRecords();
	void SetUserEquipmentRecords(UserEquipmentRecords* v);
	UserEquipmentRecord* GetUserEquipmentRecord(int idUE);                         //�õ��û���¼

	vector<UserEquipment*>* UESieving();                                          //�����û�ɸѡ
	void Scheduling_and_DL_RB_Allocation(vector<UserEquipment*>* m_UEs);          //������Դ����
	vector<PacketBurst*>*  PacketSchedule(vector<UserEquipment*>* m_UEs);       //���ɷ������ݰ� 
	void Precoding_Generate(vector<UserEquipment*>* m_UEs);                   //����Ԥ����


	void SetLtePhy(HenbLtePhy *phy);                             //����Lte�����
	HenbLtePhy* GetLtePhy();

	void SetLteMac(LteMac *mac);                            //mac��
	LteMac* GetLteMac();

	void SendPacketBurst();                  //����
	//void ReceivePacketBurst(PacketBurst* p);



	void Print();

private:

	HenbLtePhy *m_henbltephy;
	LteMac * m_ltemac;
	UserEquipmentRecords *m_userEquipmentRecords;

	int Index_sector;     //��������
};

#endif /* HeNODEB_H_ */
