


#ifndef NETWORKNODE_H_
#define NETWORKNODE_H_

#include <vector>
#include <cmath>
#include"position.h"

class NetworkNode {
public:
	enum NodeType                      //节点类型
	{
		TYPE_ENODEB,                   //宏基站
		TYPE_UE,                       //设备
		TYPE_AP,                       //AP
		TYPE_HENODEB,                  //小基站	
	};

	NetworkNode();
	virtual ~NetworkNode();

	void Destroy ();

	void SetIDNetworkNode (int id);
	int GetIDNetworkNode ();


	void MakeActive ();

	void Setposition(Position * position);
	Position * Getposition() ;

	void SetNodeType(NodeType type);
	NodeType GetNodeType() ;

	void SetPostate(Positionstate a);
	Positionstate GetPostate();

private:
	Positionstate m_positionstate;              //位置所在环境：室内/室外/车辆
	NodeType m_nodeType;                        //节点类型
	int m_idNetworkNode;                        //节点ID
    Position *m_AbsolutePosition;               //节点坐标


};


#endif /* NETWORKNODE_H_ */
