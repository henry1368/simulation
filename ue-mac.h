
#ifndef UE_MAC_H_
#define UE_MAC_H_
/*
����û���mac����Ҫ��Ϊ��wifi���ŵ����뾺����ƣ�����lte����ʱ���������У�
��˹���lte�����е��Ȳ����漰
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
	NetworkNode* m_device;       //�����豸
	DcfState* m_dcfstate;        //�û����˱�״̬
};
#endif