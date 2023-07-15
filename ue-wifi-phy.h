
#ifndef UE_WIFI_PHY_H_
#define UE_WIFI_PHY_H_

#include "wifi-phy.h"
#include"wifi-bandwidth-manager.h"
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
class CMatrix;

class UeWifiPhy :public WifiPhy {
public:
	UeWifiPhy();
	virtual ~UeWifiPhy();

	void StartRx(PacketBurst* p);                            //�û������հ�
    double* SINR_measure();                                  //����MMSE���մ������SINR

	void CreateAndSendFeedbacks(double sinr[],double m_eesmSINR);
	vector<int> CreateCqiFeedbacks(double m_eesmSINR);
	vector<int> CreatePMIFeedbacks(double sinr[]);             //����
	void SendFeedbacks(vector<int> cqi, vector<int> pmi);


	double m_measuredSinr    [RBs_FOR_WIFI];        //ÿ��Ƶ���ϵ�SINR���������߶�����Ч��  
	int  m_channelsForRx [Nr][RBs_FOR_WIFI];        //ÿ���������Ƶ����Դ      
	int  num_data_streams    [RBs_FOR_WIFI];        //ÿ��Ƶ���ϵĸ��û�������
	MatrixXcd precode_matrix [RBs_FOR_WIFI];        //ÿƵ���ϵ��û�Ԥ�������
	MatrixXd  Nss_power      [RBs_FOR_WIFI];        //ÿ��Ƶ���ϵĸ��û������ʷ������
	int  Nss;                                                   //����
};

#endif 
