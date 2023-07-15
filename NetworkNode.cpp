


#include "NetworkNode.h"
#include "stdlib.h"
#include "position.h"

NetworkNode::NetworkNode()
{
  m_AbsolutePosition = NULL;
}

NetworkNode::~NetworkNode()
{
	Destroy();
}

void
NetworkNode::Destroy ()
{
	delete m_AbsolutePosition;
}

void
NetworkNode::SetIDNetworkNode (int id)
{
  m_idNetworkNode = id;
}

int
NetworkNode::GetIDNetworkNode () 
{
  return m_idNetworkNode;
}


void 
NetworkNode::Setposition(Position * position)
{
	m_AbsolutePosition = position;
}

Position *
NetworkNode::Getposition()
{
	return m_AbsolutePosition;
}


void
NetworkNode::SetNodeType(NetworkNode::NodeType type)
{
	m_nodeType = type;

}

NetworkNode::NodeType
NetworkNode::GetNodeType() 
{
	return m_nodeType;
}

void
NetworkNode::SetPostate(Positionstate a)
{
	m_positionstate = a;
}

Positionstate
NetworkNode::GetPostate()
{
	return m_positionstate;
}
