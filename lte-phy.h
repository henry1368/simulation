#ifndef LTE_PHY_H_
#define LTE_PHY_H_

class Interference;
class NetworkNode;
class lte_BandwidthManager;
class UserEquipment;
class PacketBurst;
#include "parameter.h"
#include <vector>

using namespace std;

class LtePhy {
public:
	LtePhy();
	virtual ~LtePhy();

	void Destroy();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice();

	void SetlteBandwidthManager(lte_BandwidthManager* s);
	lte_BandwidthManager* GetlteBandwidthManager();

	void SetInterference(Interference* i);
	Interference* GetInterference();

	void SetTxPower(double p);
	double GetTxPower(void);

private:
	NetworkNode* m_device;

	lte_BandwidthManager* m_ltebandwidthManager; 

	Interference *m_interference;

	double m_txPower;
};

#endif 
