
#ifndef UE_MAC_H_
#define UE_MAC_H_
/*
设计用户的mac层主要是为了wifi中信道接入竞争设计，由于lte中暂时不考虑上行，
因此关于lte中上行调度并无涉及
*/

#include "DcfState.h"
#include "NetworkNode.h"
class UeMac{
public:
	UeMac();
	virtual ~UeMac();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice();

	DcfState* GetDcfstate();
	void SetDcfstate(DcfState* dcfstate);


private:
	NetworkNode* m_device;       //所属设备
	DcfState* m_dcfstate;        //用户的退避状态
};
#endif