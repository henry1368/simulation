
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

	void SetRoomheight(double h);     //边缘
	double GetRoomheight();

	int GetIdBuilding ();
	void SetIdBuilding (int id);     //建筑号

	void SetFloors (int floors);     //楼层数
	int GetFloors();

	void SetSide (double* side);     //边缘
	double* GetSide ();	

	void SetCenterPosition(Position* position);  //中心位置
	Position* GetCenterPosition ();
	
	bool IsInThisBuilding (NetworkNode* s);  //判断某个用户是否在该大楼内

private:	 
	int m_idBuilding;
	int m_nbFloors;
	double m_roomheight;
	double* m_side;
	Position* m_centerPosition;
};


#endif 
