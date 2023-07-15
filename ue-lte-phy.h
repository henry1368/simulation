#ifndef UE_LTE_PHY_H_
#define UE_LTE_PHY_H_

#include "lte-phy.h"
#include"lte-bandwidth-manager.h"
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
class CMatrix;

class UeLtePhy :public LtePhy {
public:
	UeLtePhy();
	virtual ~UeLtePhy();

	virtual void TxPowerAllocation(double p);

	void CreateAndSendFeedbacks(double sinr[], double m_eesmSINR);  //���CQI���Ӵ�PMI
	vector<int> CreateCqiFeedbacks(double m_eesmSINR);
	vector<int> CreatePMIFeedbacks(double sinr[]);
	void SendFeedbacks(vector<int> cqi, vector<int> pmi);

	void StartRx(PacketBurst* p);                            //�û������հ�

	double* SINR_measure();                                  //����MMSE���մ������SINR

	//	vector<MatrixXcf>  precode_matrix;                       //���û���Ԥ����     ÿ���û�����Ƶ��������Ԥ�������      (��AP�������������������û�������) 
	//	int  m_mcsIndexForRx  [Nr];                              //�洢MCS  ��0-9��            ��ͬ����Ӧ�Ų�ͬ��MCS 	


	double m_measuredSinr[RBs_FOR_LTE];           //ÿ��Ƶ���ϵ�SINR���������߶�����Ч��  
	int  m_channelsForRx[Nr][RBs_FOR_LTE];        //ÿ���������Ƶ����Դ      
	int  num_data_streams[RBs_FOR_LTE];           //ÿ��Ƶ���ϵĸ��û�������
	MatrixXcd precode_matrix[RBs_FOR_LTE];        //ÿƵ���ϵ��û�Ԥ�������
	MatrixXd  Nss_power[RBs_FOR_LTE];             //ÿ��Ƶ���ϵĸ��û������ʷ������
	int  Nss;                                                //��������
};

#endif 

