

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

		UserEquipment *m_UE;                              //用户
		vector<vector<int> > m_cqiFeedback;               //CQI反馈
		vector<vector<int> > m_PMI;                       //PMI

        int m_Mcs;                                        //调制编码方式
		vector<MacQueue*> * m_macqueue;                   //对应用户的缓存队列(一个业务对应一个缓存队列)


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

	

	enum DLSchedulerType                                                            //下行调度方式
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
	ENodeB(int idElement,  Position *s);                               //生成基站         

	virtual ~ENodeB();

    void RegisterUserEquipment (UserEquipment *UE);                                 //注册用户
    void DeleteUserEquipment (UserEquipment *UE);                                   //删除用户
                                           
    void CreateUserEquipmentRecords ();                                          //用户记录表
    void DeleteUserEquipmentRecords ();                                          //删除用户记录表  

    UserEquipmentRecords* GetUserEquipmentRecords ();   
	void SetUserEquipmentRecords(UserEquipmentRecords* v);
    UserEquipmentRecord* GetUserEquipmentRecord (int idUE);                       //得到用户记录
        
    vector<UserEquipment*>* UESieving();                                         //进行用户筛选



	void (*Scheduling_and_DL_RB_Allocation)(vector<UserEquipment*>* m_UEs, EnbLtePhy* X);          //下行资源分配




	vector<PacketBurst*>*  PacketSchedule(vector<UserEquipment*>* m_UEs);         //生成发送数据包 

	void Precoding_Generate(vector<UserEquipment*>* m_UEs);                       //生成预编码
	
	
	void SetLtePhy(EnbLtePhy *phy);                             //关于Lte物理层
	EnbLtePhy* GetLtePhy();

	void SetLteMac(LteMac *mac);                            //mac层
	LteMac* GetLteMac();

	void SendPacketBurst();                  //发包

	void Print ();

private:

	EnbLtePhy  *m_enbltephy;
	LteMac * m_ltemac;
	UserEquipmentRecords *m_userEquipmentRecords;
	

};

#endif
