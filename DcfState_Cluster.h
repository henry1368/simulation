
#ifndef DCFSTATE_CLUSTER_H_
#define DCFSTATE_CLUSTER_H_
#include "vector"
class DcfState;
class AP;
class NetworkNode;
class UserEquipment;
using namespace std;
class DcfState_Cluster                    //每一个由AP和连接AP的用户状态组成的竞争簇
{
public:
	DcfState_Cluster(AP* ap);
	~DcfState_Cluster();
	void StartCop();                       //开始
	void EndDCF();                         //结束
	void  update_cluster_member();         //更新当前簇的竞争成员
	NetworkNode*  Compete();               //竞争过程，得出竞争结果
	void update_cluster_info();            //更新该簇的全部用户



	typedef vector<DcfState*> State;       //将所有的竞争用户状态放到一起，退避时隙最小者将获得信道接入权
	State* m_state;

    AP * local_ap;                         //该簇中的AP

//	vector<UserEquipment*> m_ap_connect_ue;   //该簇所


	double m_lastNavStart;        //开始退避时刻
	double m_lastNavDuration;     //退避总时长
	double m_TxStart;            //开始传输时间
	double m_TxDuration;

	double m_RxStart;           //开始接收时间
	double m_RxDuration;
	double m_RxEnd;             //接收结束时间 
	bool   m_lastRxReceiveOk;
	
};
#endif