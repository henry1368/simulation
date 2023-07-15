#ifndef HENB_LTE_PHY_H_
#define HENB_LTE_PHY_H_

#include "lte-phy.h"
#include "lte-bandwidth-manager.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class HenbLtePhy :public LtePhy {

public:
	HenbLtePhy();
	virtual ~HenbLtePhy();

	void TxPowerAllocation( vector<UserEquipment*>* Dst_UEs);      //功率配置接口

	void StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets);     //下行henb发包

	int  m_num_tx_streams[RBs_FOR_LTE];       //每频点上的总发射流数(<=Nt)

	MatrixXcd  precode_matrix[RBs_FOR_LTE];      //每频点上的所有用户 预编码 W=[H1 H2....Hn]

	MatrixXd   allNss_power[RBs_FOR_LTE];            //每个频点上的所有用户所有流功率分配

};

#endif 
