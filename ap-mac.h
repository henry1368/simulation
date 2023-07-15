
#ifndef AP_MAC_H_
#define AP_MAC_H_
#include "DcfState.h"
#include "NetworkNode.h"
using namespace std;
class ApMac{
public:
	ApMac();
	virtual ~ApMac();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice();

	DcfState* GetDcfstate();
	void SetDcfstate(DcfState* dcfstate);



private:
	NetworkNode* m_device;       //所属设备
	DcfState* m_dcfstate;        //用户的退避状态
	
	               //业务模型
};
#endif