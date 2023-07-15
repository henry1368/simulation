


#ifndef NETWORKNODE_H_
#define NETWORKNODE_H_

#include <vector>
#include <cmath>
#include"position.h"

class NetworkNode {
public:
	enum NodeType                      //�ڵ�����
	{
		TYPE_ENODEB,                   //���վ
		TYPE_UE,                       //�豸
		TYPE_AP,                       //AP
		TYPE_HENODEB,                  //С��վ	
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
	Positionstate m_positionstate;              //λ�����ڻ���������/����/����
	NodeType m_nodeType;                        //�ڵ�����
	int m_idNetworkNode;                        //�ڵ�ID
    Position *m_AbsolutePosition;               //�ڵ�����


};


#endif /* NETWORKNODE_H_ */
