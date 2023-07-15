#include "ap-mac.h"




ApMac::ApMac()
{
	SetDcfstate(new DcfState());
}

ApMac::~ApMac()
{
	m_dcfstate = NULL;
	delete m_dcfstate;

}


void
ApMac::SetDevice(NetworkNode* d)
{
        m_device = d;
		m_dcfstate->Belong_Node = d;
}


NetworkNode*                                   //�����豸
ApMac::GetDevice()
{
return m_device;
}



DcfState*
ApMac::GetDcfstate()
{
	return m_dcfstate;

}

void
ApMac::SetDcfstate(DcfState* dcfstate)        //�˱�״̬
{
	m_dcfstate = dcfstate;
}


