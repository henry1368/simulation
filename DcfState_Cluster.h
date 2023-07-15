
#ifndef DCFSTATE_CLUSTER_H_
#define DCFSTATE_CLUSTER_H_
#include "vector"
class DcfState;
class AP;
class NetworkNode;
class UserEquipment;
using namespace std;
class DcfState_Cluster                    //ÿһ����AP������AP���û�״̬��ɵľ�����
{
public:
	DcfState_Cluster(AP* ap);
	~DcfState_Cluster();
	void StartCop();                       //��ʼ
	void EndDCF();                         //����
	void  update_cluster_member();         //���µ�ǰ�صľ�����Ա
	NetworkNode*  Compete();               //�������̣��ó��������
	void update_cluster_info();            //���¸ôص�ȫ���û�



	typedef vector<DcfState*> State;       //�����еľ����û�״̬�ŵ�һ���˱�ʱ϶��С�߽�����ŵ�����Ȩ
	State* m_state;

    AP * local_ap;                         //�ô��е�AP

//	vector<UserEquipment*> m_ap_connect_ue;   //�ô���


	double m_lastNavStart;        //��ʼ�˱�ʱ��
	double m_lastNavDuration;     //�˱���ʱ��
	double m_TxStart;            //��ʼ����ʱ��
	double m_TxDuration;

	double m_RxStart;           //��ʼ����ʱ��
	double m_RxDuration;
	double m_RxEnd;             //���ս���ʱ�� 
	bool   m_lastRxReceiveOk;
	
};
#endif