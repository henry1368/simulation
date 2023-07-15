#include <iostream>
#include "position.h"

Position::Position()
{
}

Position::~Position()
{}

Position::Position(double x, double y,double z)
{
	m_posX = x;
	m_posY = y;
	m_posZ = z;
}

double
Position::GetPositionX()
{
	return m_posX;
}

double
Position::GetPositionY()
{
	return m_posY;
}

double
Position::GetPositionZ()
{
	return m_posZ;
}


void
Position::SetPositionX(double x)
{
	m_posX = x;
}

void
Position::SetPositionY(double y)
{
	m_posY = y;
}

void
Position::SetPositionZ(double z)
{
	m_posZ = z;
}


void
Position::Print()
{
  std::cout << "\t Position: "
	  "x = " << GetPositionX() <<
	  ", y = " << GetPositionY()<<
	  ", y = " << GetPositionZ() << std::endl;
}



static double distance(Position* a, Position* b)       //Á½µã¼ä¾àÀë
{
	double x = a->GetPositionX() - b->GetPositionX();
	double y = a->GetPositionY() - b->GetPositionY();
	double z = a->GetPositionZ() - b->GetPositionZ();
	return  sqrt(x*x + y*y + z*z);

}