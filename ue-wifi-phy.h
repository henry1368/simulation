
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

	void StartRx(PacketBurst* p);                            //用户下行收包
    double* SINR_measure();                                  //采用MMSE接收处理计算SINR

	void CreateAndSendFeedbacks(double sinr[],double m_eesmSINR);
	vector<int> CreateCqiFeedbacks(double m_eesmSINR);
	vector<int> CreatePMIFeedbacks(double sinr[]);             //反馈
	void SendFeedbacks(vector<int> cqi, vector<int> pmi);


	double m_measuredSinr    [RBs_FOR_WIFI];        //每载频点上的SINR（单流或者多流等效）  
	int  m_channelsForRx [Nr][RBs_FOR_WIFI];        //每个流分配的频率资源      
	int  num_data_streams    [RBs_FOR_WIFI];        //每个频点上的该用户的流数
	MatrixXcd precode_matrix [RBs_FOR_WIFI];        //每频点上的用户预编码矩阵
	MatrixXd  Nss_power      [RBs_FOR_WIFI];        //每个频点上的该用户流功率分配矩阵
	int  Nss;                                                   //流数
};

#endif 
