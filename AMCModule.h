#ifndef AMCModule_H_
#define AMCModule_H_

#include <vector>
/*
 *  自适应编码调制
 */

class AMCModule {
public:
	AMCModule();
	virtual ~AMCModule();

	int
	GetCQIFromEfficiency (double sinr);
	int
	GetMCSIndexFromEfficiency(double spectralEfficiency);
	int
	GetMCSFromCQI (int cqi);
	int
	GetCQIFromMCS (int mcs);
	int
	GetTBSizeFromMCS (int mcs);
	int
	GetTBSizeFromMCS(int mcs, int *RB, int array_size);
	double
	GetEfficiencyFromCQI (int cqi);
	int
    GetCQIFromSinr (double sinr);
	double
	GetSinrFromCQI (int cqi);

};

#endif /* AMCModule_H_ */
