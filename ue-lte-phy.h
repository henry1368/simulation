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

	void CreateAndSendFeedbacks(double sinr[], double m_eesmSINR);  //宽带CQI，子带PMI
	vector<int> CreateCqiFeedbacks(double m_eesmSINR);
	vector<int> CreatePMIFeedbacks(double sinr[]);
	void SendFeedbacks(vector<int> cqi, vector<int> pmi);

	void StartRx(PacketBurst* p);                            //用户下行收包

	double* SINR_measure();                                  //采用MMSE接收处理计算SINR

	//	vector<MatrixXcf>  precode_matrix;                       //该用户的预编码     每个用户：【频点数】×预编码矩阵      (【AP发射天线数】×【该用户流数】) 
	//	int  m_mcsIndexForRx  [Nr];                              //存储MCS  【0-9】            不同流对应着不同的MCS 	


	double m_measuredSinr[RBs_FOR_LTE];           //每载频点上的SINR（单流或者多流等效）  
	int  m_channelsForRx[Nr][RBs_FOR_LTE];        //每个流分配的频率资源      
	int  num_data_streams[RBs_FOR_LTE];           //每个频点上的该用户的流数
	MatrixXcd precode_matrix[RBs_FOR_LTE];        //每频点上的用户预编码矩阵
	MatrixXd  Nss_power[RBs_FOR_LTE];             //每个频点上的该用户流功率分配矩阵
	int  Nss;                                                //发射流数
};

#endif 

