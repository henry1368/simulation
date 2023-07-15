
#ifndef CELL_H_
#define CELL_H_

#include <stdint.h>
#include <iostream>
#include "position.h"

using namespace std;

class Cell {
private: 
	int m_idCell;                      //编号
	double m_radius;			       // 半径Km
	double m_minDistance;	           //最小距离Km
	Position *m_CellCenterPosition;    //中心位置

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
