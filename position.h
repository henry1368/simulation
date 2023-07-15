

#ifndef POSITION_H_
#define POSITION_H_


//static double distance(Position* a, Position* b);

enum Positionstate              //所在位置分类
{
	m_Indoor,
	m_Outdoor,
	m_Vehicle,
};

class Position
{

public:
	Position();
	Position(double x,double y,double z);
    ~Position();
	double GetPositionX();
	double GetPositionY();
	double GetPositionZ();
	void SetPositionX(double X);
	void SetPositionY(double Y);
	void SetPositionZ(double Z);
	void Print();

private:
	double m_posX;
	double m_posY;
	double m_posZ;

};

#endif 
