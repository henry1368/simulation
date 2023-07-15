
#ifndef BUILDING_H_
#define BUILDING_H_

class Position;
class UserEquipment;
class NetworkNode;

class Building {
public:

	Building ();
	Building(int idBuilding,
		int nbFloors,
		double roomheight,
		double side_X,
		double side_Y,
		double center_X,
		double center_Y);

	~Building();

	void SetRoomheight(double h);     //��Ե
	double GetRoomheight();

	int GetIdBuilding ();
	void SetIdBuilding (int id);     //������

	void SetFloors (int floors);     //¥����
	int GetFloors();

	void SetSide (double* side);     //��Ե
	double* GetSide ();	

	void SetCenterPosition(Position* position);  //����λ��
	Position* GetCenterPosition ();
	
	bool IsInThisBuilding (NetworkNode* s);  //�ж�ĳ���û��Ƿ��ڸô�¥��

private:	 
	int m_idBuilding;
	int m_nbFloors;
	double m_roomheight;
	double* m_side;
	Position* m_centerPosition;
};


#endif 
