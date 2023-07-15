


#ifndef AP_WIFI_PHY_H_
#define AP_WIFI_PHY_H_

#include "wifi-phy.h"
#include"wifi-bandwidth-manager.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
class CMatrix;



class APWifiPhy :public WifiPhy {
public:
	APWifiPhy();
	virtual ~APWifiPhy();

	void TxPowerAllocation(vector<UserEquipment*>* Dst_UEs);      //�������ýӿ�

	void StartTx(vector<UserEquipment*>* Dst_UEs, vector<PacketBurst*>* Tx_Packets);     //����AP����

	int  m_num_tx_streams[RBs_FOR_WIFI];       //ÿƵ���ϵ��ܷ�������(<=Nt)

	MatrixXcd  precode_matrix[RBs_FOR_WIFI];      //ÿƵ���ϵ������û� Ԥ���� W=[H1 H2....Hn]

	MatrixXd   allNss_power[RBs_FOR_WIFI];            //ÿ��Ƶ���ϵ������û����������ʷ���

};

#endif

