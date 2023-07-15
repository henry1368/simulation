
#ifndef INTERFERENCE_H_
#define INTERFERENCE_H_                  //����������ŵ�
#include "parameter.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class UserEquipment;

class Interference {
public:
	Interference();
	virtual ~Interference();

    void  WIFI_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>*AP_interference_matrix);       //��������AP����

	void  LTE_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>* Enb_interference_matrix);      //�����������վ��С��վ����

};

#endif 
