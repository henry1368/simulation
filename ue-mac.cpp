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
	GetDcfstate()->Belong_Node = d;              //���������豸
}


NetworkNode*                                   //�����豸
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
UeMac::SetDcfstate(DcfState* dcfstate)        //�˱�״̬
{
	m_dcfstate = dcfstate;
}


