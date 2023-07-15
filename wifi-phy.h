#ifndef WIFI_PHY_H_
#define WIFI_PHY_H_

#include <vector>
#include "parameter.h"
using namespace std;

class Interference;
class NetworkNode;
class UserEquipment;
class wifi_BandwidthManager;
class PacketBurst;

class WifiPhy {
public:
	WifiPhy();
	virtual ~WifiPhy();

	void Destroy();

	void SetwifiBandwidthManager(wifi_BandwidthManager* s);
	wifi_BandwidthManager* GetwifiBandwidthManager();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice();

	void SetInterference(Interference* i);
	Interference* GetInterference();

	void SetTxPower(double p);
	double GetTxPower(void);

private:
	NetworkNode* m_device;

	wifi_BandwidthManager* m_wifibandwidthManager;       //´ø¿í¹ÜÀí

	Interference * m_interference;

	double m_txPower;

};

#endif 
