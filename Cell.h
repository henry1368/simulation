
#ifndef CELL_H_
#define CELL_H_

#include <stdint.h>
#include <iostream>
#include "position.h"

using namespace std;

class Cell {
private: 
	int m_idCell;                      //���
	double m_radius;			       // �뾶Km
	double m_minDistance;	           //��С����Km
	Position *m_CellCenterPosition;    //����λ��

public:
	Cell();
	Cell(int idCell,
		 double r,
		 double minDistance,
		 double X,
		 double Y);
	 ~Cell();

	void
	SetIdCell (int idCell);
	int
	GetIdCell () const;
	void
	SetRadius (double r);
	double
	GetRadius () const;
	void
	SetMinDistance (double minDistance);
	double
	GetMinDistance () const;
	void
	SetCellCenterPosition (Position *position);
	Position*
	GetCellCenterPosition () const;


	
	void
	Print ();
};

#endif 
