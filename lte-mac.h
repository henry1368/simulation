#ifndef LTE_MAC_H_
#define LTE_MAC_H_
#include "NetworkNode.h"
//ou
#include "AMCModule.h"
//........

using namespace std;
class LteMac{
public:
	LteMac();
	virtual ~LteMac();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice();

	
	//ou
	AMCModule* GetAmcModule() const;
	//............



private:
	NetworkNode* m_device;       //À˘ Ù…Ë±∏
	//ou
	AMCModule* m_amcModule;
	//......
};
#endif