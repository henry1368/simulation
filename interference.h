
#ifndef INTERFERENCE_H_
#define INTERFERENCE_H_                  //用来计算干扰的
#include "parameter.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class UserEquipment;

class Interference {
public:
	Interference();
	virtual ~Interference();

    void  WIFI_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>*AP_interference_matrix);       //计算其它AP干扰

	void  LTE_ComputeInterference(UserEquipment *ue, Matrix<complex<double>, Nr, Nr>* Enb_interference_matrix);      //计算其它宏基站、小基站干扰

};

#endif 
