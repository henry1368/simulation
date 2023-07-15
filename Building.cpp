#include "Building.h"
#include "position.h"
#include "NetworkNode.h"

Building::Building ()           //建筑为长方形或者正方形
{}

Building::Building (int idBuilding,
	     	 	    int nbFloors,
					double roomheight,
	     	 	    double side_X,
	     	 	    double side_Y,
	     	 	    double center_X,
	     	 	    double center_Y)
{
	
	m_idBuilding = idBuilding; 
	m_nbFloors = nbFloors; 
	m_roomheight = roomheight;
	m_side = new double[2];
	m_side [0] = side_X;        //建筑边缘距离中心位置水平距离
	m_side [1] = side_Y;        //竖直距离
	Position *pos = new Position(center_X, center_Y, 0);
	m_centerPosition = pos;

}

Building::~Building()
{
  delete m_centerPosition;
}

void
Building::SetIdBuilding (int id)
{
  m_idBuilding = id;
}

int
Building::GetIdBuilding()
{
	return m_idBuilding;
}


void
Building::SetFloors (int floors)
{
  m_nbFloors = floors;
}

int
Building::GetFloors()
{
	return m_nbFloors;
}


void
Building::SetRoomheight(double h)     //边缘
{
	m_roomheight = h;
}

double 
Building::GetRoomheight()
{
	return m_roomheight;
}

void
Building::SetSide (double* side)
{
  m_side = side;
}

double*
Building::GetSide()
{
	return m_side ;
}


void
Building::SetCenterPosition(Position* position)
{
  m_centerPosition = position;
}


Position*
Building::GetCenterPosition (void)
{
  return m_centerPosition;
}



bool
Building::IsInThisBuilding (NetworkNode* s)
{
	double x = m_centerPosition->GetPositionX() - s->Getposition()->GetPositionX();
	double y = m_centerPosition->GetPositionY() - s->Getposition()->GetPositionY();
	
	if ((x*x <= (GetSide()[0]) * (GetSide()[0])) && (y*y <= (GetSide()[1]) * (GetSide()[1])))		  
	  return true;
	else 
	return false;
}




