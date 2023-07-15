
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
	NetworkNode* m_device;       //�����豸
	DcfState* m_dcfstate;        //�û����˱�״̬
	
	               //ҵ��ģ��
};
#endif