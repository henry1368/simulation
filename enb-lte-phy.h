#ifndef ENB_LTE_PHY_H_
#define ENB_LTE_PHY_H_

#include "lte-phy.h"
#include "lte-bandwidth-manager.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class CMatrix;

class EnbLtePhy :public LtePhy {
public:
	EnbLtePhy();
	virtual ~EnbLtePhy();

	void TxPowerAllocation(vector<UserEquipment*>* Dst_UEs);      //功率配置接口

	void StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets);     //下行enb发包

	int  m_num_tx_streams[RBs_FOR_LTE];       //每频点上的总发射流数(<=Nt)

	MatrixXcd  precode_matrix[RBs_FOR_LTE];      //每频点上的所有用户 预编码 W=[H1 H2....Hn]

	MatrixXd   allNss_power[RBs_FOR_LTE];            //每个频点上的所有用户所有流功率分配

};

#endif /* ENB_LTE_PHY_H_ */
