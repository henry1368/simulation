#ifndef  WIFI_CHANNEL_H_
#define  WIFI_CHANNEL_H_                      //����û����վ����AP֮��ϵ������
#include <vector>
#include <complex>
#include "parameter.h"
#include "wifi-bandwidth-manager.h"
#include<cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class UserEquipment;
class WifiPropagationLossModel;


class WifiChannel
{

public:
	WifiChannel();    
	~WifiChannel();

	UserEquipment* GetUe();
	void SetUe(UserEquipment* a);


	WifiPropagationLossModel * GetWifiPropagationLossModel();
	void SetWifiPropagationLossModel(WifiPropagationLossModel*b);

	double    APthetaBS[nb_totalAP];                           
	double    APthetaMS[nb_totalAP];          //����Ǻ͸�����
	double   APelevation[nb_totalAP];

	double    AP_path_loss[nb_totalAP];        //·��
	double    AP_shadow_loss[nb_totalAP];      //��Ӱ˥��
	double    AP_penetration_loss[nb_totalAP];  //��͸���
	double    AP_antennaGain[nb_totalAP];       //��������
	bool      AP_isLOS[nb_totalAP];             //������
	double    AP_LSPs[nb_totalAP][5];           //��չ����
	
	double AP_power_loss[nb_totalAP];
	MatrixXcd channel_APcoefficients[nb_totalAP][RBs_FOR_WIFI];    //������û�������AP���ڲ�ͬ��Ƶ���ϵ��ŵ�����(Nr��Nt)

private:

	WifiPropagationLossModel * m_WifiPropagationLossModel;
	UserEquipment* m_ue;             //�����û�


};
#endif

