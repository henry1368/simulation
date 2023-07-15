

#include "Cell.h"
#include"position.h"

Cell::Cell()
{}

Cell::Cell(int idCell,
		   double r,
		   double minDistance,
		   double X,
		   double Y)
{
  m_idCell = idCell;
  m_radius = r;
  m_minDistance = minDistance;
  Position *CellPosition =new Position(X, Y);
  SetCellCenterPosition(CellPosition);
  
}

Cell::~Cell()
{
  delete m_CellCenterPosition;
}

void
Cell::SetIdCell (int idCell)
{
  m_idCell = idCell;
}

int
Cell::GetIdCell () const
{
  return m_idCell;
}

void
Cell::SetRadius (double r)
{
  m_radius = r;
}

double
Cell::GetRadius () const
{
  return m_radius;
}

void
Cell::SetMinDistance (double minDistance)
{
  m_minDistance = minDistance;
}

double
Cell::GetMinDistance () const
{
  return m_minDistance;
}

void
Cell::SetCellCenterPosition(Position *position)
{
	m_CellCenterPosition = position;
}

Position *
Cell::GetCellCenterPosition () const
{
	return m_CellCenterPosition;
}


void
Cell::Print ()
{
  std::cout <<
      "\n\t m_idCell = " << m_idCell <<
      "\n\t m_radius = " << m_radius <<
	  "\n\t m_minDistance = " <<  m_minDistance <<
	  std::endl;

  GetCellCenterPosition ()->Print();
}
