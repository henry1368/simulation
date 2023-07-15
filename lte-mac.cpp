#include "lte-mac.h"

LteMac::LteMac()
{
}

LteMac::~LteMac()
{
}


void
LteMac::SetDevice(NetworkNode* d)
{
	m_device = d;
}


NetworkNode*                                   //所连设备
LteMac::GetDevice()
{
	return m_device;
}





//ou add
AMCModule*
LteMac::GetAmcModule() const
{
	return m_amcModule;
}
