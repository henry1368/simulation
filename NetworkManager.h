

#ifndef NetworkManager_H_
#define NetworkManager_H_

#include <stdint.h>
#include <vector>
#include "parameter.h"

class NetworkNode;
class AP;
class ENodeB;
class UserEquipment;
class Position;
class HeNodeB;
class Building;


class NetworkManager {
private:
	std::vector<Building*> *m_BuildingContainer;

	std::vector<ENodeB*> *m_ENodeBContainer;

	std::vector<HeNodeB*> *m_HeNodeBContainer;

	std::vector<UserEquipment*> *m_UserEquipmentContainer;

	std::vector<AP*> *m_APContainer;

	NetworkManager();

	static NetworkManager *ptr;

public:




	virtual ~NetworkManager();

	static NetworkManager*
	Init ()
	  {
		if (ptr==NULL)
	      {
		    ptr = new NetworkManager;
	   	  }
		return ptr;
	  }

	/*
	 * //获取元素集
	 */

	std::vector<AP*>*
	GetAPContainer ();

	std::vector<ENodeB*>*
		GetENodeBContainer();

	std::vector<UserEquipment*>*
		GetUserEquipmentContainer();

	std::vector<HeNodeB*>*
		GetHeNodeBContainer();

	std::vector<Building*>*
		GetBuildingContainer();
	
	void updateLinkInfo();                         //更新系统内用户的位置，切换及更新位置

//	void  update_Ksi(int m_scene);                //根据空间相关性进行空间滤波重新计算关于信道参数Ski
//	double Ski[nb_totalUe * 3* nb_cell][5];    //所有链路相关性参数Ski
};


#endif 



