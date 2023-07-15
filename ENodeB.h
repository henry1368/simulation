

#ifndef ENODEB_H_
#define ENODEB_H_

#include "NetworkNode.h"
#include "FUN.h"


class UserEquipment;
class EnbLtePhy;
class LteMac;
class PacketBurst;
class MacQueue;

using namespace std;

class ENodeB : public NetworkNode {
public:
	struct UserEquipmentRecord
	  {
		UserEquipmentRecord ();        //
		virtual ~UserEquipmentRecord ();
		UserEquipmentRecord (UserEquipment *UE);

		UserEquipment *m_UE;                              //�û�
		vector<vector<int> > m_cqiFeedback;               //CQI����
		vector<vector<int> > m_PMI;                       //PMI

        int m_Mcs;                                        //���Ʊ��뷽ʽ
		vector<MacQueue*> * m_macqueue;                   //��Ӧ�û��Ļ������(һ��ҵ���Ӧһ���������)


		void SetUE (UserEquipment *UE);
        UserEquipment* GetUE () const;
      
		void SetCQI(vector<vector<int> > cqi);
		vector<vector<int> > GetCQI() const;
		
		void SetPMI(vector<vector<int> > pmi);
		vector<vector<int> > GetPMI() const;

		void SetMcs (int mcs);
		int GetMcs ();

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
	ENodeB ();
	ENodeB(int idElement,  Position *s);                               //���ɻ�վ         

	virtual ~ENodeB();

    void RegisterUserEquipment (UserEquipment *UE);                                 //ע���û�
    void DeleteUserEquipment (UserEquipment *UE);                                   //ɾ���û�
                                           
    void CreateUserEquipmentRecords ();                                          //�û���¼��
    void DeleteUserEquipmentRecords ();                                          //ɾ���û���¼��  

    UserEquipmentRecords* GetUserEquipmentRecords ();   
	void SetUserEquipmentRecords(UserEquipmentRecords* v);
    UserEquipmentRecord* GetUserEquipmentRecord (int idUE);                       //�õ��û���¼
        
    vector<UserEquipment*>* UESieving();                                         //�����û�ɸѡ



	void (*Scheduling_and_DL_RB_Allocation)(vector<UserEquipment*>* m_UEs, EnbLtePhy* X);          //������Դ����




	vector<PacketBurst*>*  PacketSchedule(vector<UserEquipment*>* m_UEs);         //���ɷ������ݰ� 

	void Precoding_Generate(vector<UserEquipment*>* m_UEs);                       //����Ԥ����
	
	
	void SetLtePhy(EnbLtePhy *phy);                             //����Lte�����
	EnbLtePhy* GetLtePhy();

	void SetLteMac(LteMac *mac);                            //mac��
	LteMac* GetLteMac();

	void SendPacketBurst();                  //����

	void Print ();

private:

	EnbLtePhy  *m_enbltephy;
	LteMac * m_ltemac;
	UserEquipmentRecords *m_userEquipmentRecords;
	

};

#endif
