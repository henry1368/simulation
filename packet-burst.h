
#ifndef PACKET_BURST_H
#define PACKET_BURST_H

#include <stdint.h>
#include <vector>
#include "Packet.h"


class PacketBurst
{
	//�����б���վ��AP��ÿ�θ�����û�һ�𷢰�

public:
	PacketBurst();
	virtual ~PacketBurst();

	PacketBurst* Copy();

	void AddPacket(Packet* packet);
	std::vector<Packet* > GetPackets() ;

	double GetNPackets() ;
	double GetSize() ;

private:
	std::vector<Packet* > m_packets;
};


#endif 
