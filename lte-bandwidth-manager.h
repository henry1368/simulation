


#ifndef LTE_BandwidthManager_H_
#define LTE_BandwidthManager_H_

#include <vector>

#define LTE_BANDWIDTH     20                  //设定lte信道带宽为10MHz
#define RBs_FOR_LTE 100                       //lte资源快个数
#define DL_LOW_FREQUENCY_BAND_LTE 2110 	         //MHz
#define DL_HIGH_FREQUENCY_BAND_LTE 2170 	     //MHz
#define DL_LOW_FREQUENCY_BAND_LTE_HENB 3399.5 	 //MHz
#define  DL_HIGH_FREQUENCY_BAND_LTE_HENB 3499.5  //MHz

class lte_BandwidthManager {
public:
	lte_BandwidthManager();
	lte_BandwidthManager(char *type);
	virtual ~lte_BandwidthManager();

	void SetDLSubChannels(std::vector<double> s);
	std::vector<double> GetDLSubChannels();

	void SetlteBandwidth(double b);
	double GetlteBandwidth();

	lte_BandwidthManager* Copy();

	void Print();

private:
	std::vector<double> m_dlSubChannels;

	double m_lteBandwidth;
	

};

#endif 
