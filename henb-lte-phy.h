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

	void TxPowerAllocation( vector<UserEquipment*>* Dst_UEs);      //�������ýӿ�

	void StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets);     //����henb����

	int  m_num_tx_streams[RBs_FOR_LTE];       //ÿƵ���ϵ��ܷ�������(<=Nt)

	MatrixXcd  precode_matrix[RBs_FOR_LTE];      //ÿƵ���ϵ������û� Ԥ���� W=[H1 H2....Hn]

	MatrixXd   allNss_power[RBs_FOR_LTE];            //ÿ��Ƶ���ϵ������û����������ʷ���

};

#endif 
