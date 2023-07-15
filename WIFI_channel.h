#ifndef  WIFI_CHANNEL_H_
#define  WIFI_CHANNEL_H_                      //算出用户与基站或者AP之间系数矩阵
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
	double    APthetaMS[nb_totalAP];          //到达角和俯仰角
	double   APelevation[nb_totalAP];

	double    AP_path_loss[nb_totalAP];        //路损
	double    AP_shadow_loss[nb_totalAP];      //阴影衰落
	double    AP_penetration_loss[nb_totalAP];  //穿透损耗
	double    AP_antennaGain[nb_totalAP];       //天线增益
	bool      AP_isLOS[nb_totalAP];             //可视性
	double    AP_LSPs[nb_totalAP][5];           //扩展参数
	
	double AP_power_loss[nb_totalAP];
	MatrixXcd channel_APcoefficients[nb_totalAP][RBs_FOR_WIFI];    //储存该用户到所有AP的在不同载频点上的信道矩阵(Nr×Nt)

private:

	WifiPropagationLossModel * m_WifiPropagationLossModel;
	UserEquipment* m_ue;             //所属用户


};
#endif

