

#include <stdint.h>
#include <vector>
#include "packet-burst.h"


PacketBurst::PacketBurst ()
{    
	 
}

PacketBurst::~PacketBurst ()
{
	for (std::vector<Packet* >::const_iterator iter = m_packets.begin(); iter != m_packets.end(); ++iter)
    {
	  delete *iter;
    }
   m_packets.clear();
}

PacketBurst*
PacketBurst::Copy ()
{
  PacketBurst* pb = new PacketBurst ();

  std::vector<Packet* > packets = GetPackets();
  std::vector<Packet* >::iterator it;

  for (it = packets.begin (); it != packets.end (); it++)
	{
	  pb->AddPacket (*it);
	}

  return pb;
}

std::vector<Packet*>
PacketBurst::GetPackets () 
{
  return m_packets;
}

void
PacketBurst::AddPacket(Packet* packet)
{
  if (packet)
    {
      m_packets.push_back (packet);
    }
}

double
PacketBurst::GetNPackets()   //返回包的个数
{
	return m_packets.size();
}

double 
PacketBurst::GetSize()       //返回整体数据大小
{
	double sum = 0;
	for (uint32_t i = 0; i < m_packets.size(); i++)
		sum += m_packets.at(i)->m_size;
	return sum;
}




