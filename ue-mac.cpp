#include "ue-mac.h"




UeMac::UeMac()
{	
	SetDcfstate(new DcfState());   
}

UeMac::~UeMac()
{
	m_dcfstate = NULL;
	delete m_dcfstate;

}


void 
UeMac::SetDevice(NetworkNode* d)
{
	m_device = d;
	GetDcfstate()->Belong_Node = d;              //设置所属设备
}


NetworkNode*                                   //所连设备
UeMac::GetDevice()
{
	return m_device;
}



DcfState*
UeMac::GetDcfstate()
{
	return m_dcfstate;

}

void
UeMac::SetDcfstate(DcfState* dcfstate)        //退避状态
{
	m_dcfstate = dcfstate;
}


