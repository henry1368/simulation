
#ifndef MOBILITY_H_
#define MOBILITY_H_

//移动模型定义了在不同场景下的用户的移动模型：相关的移动速度、方向和位置更新函数等
/*
设定用户的移动模型共3种：
1.静止状态
2.随机移动
3.固定轨迹

*/
//其中第2种移动模型依照M2135.Table8-4 相关规定定义用户的移动速度和方向

#include <stdlib.h>
#include"position.h"


class Mobility {                                 
public:
	Mobility(char* str, Positionstate p,int m_move_index);  //场景和移动模型
	virtual ~Mobility();

	int m_Move;                            //移动模型   (静止、随机、定轨)
	double m_speed;			 		       //速度 Km/h
	double m_speedDirection;               //移动方向

	double m_last_update_time;                //上次更新位置时间
	Position* m_last_update_lte_position;     //LTE中上次发生大尺度变化的位置
	Position* m_last_update_wifi_position;    //WIFI中上次发生大尺度变化的位置

	void UpdatePosition(Position* pos);       


};

#endif 
