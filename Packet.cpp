
#include "Packet.h"
#include "simulator.h"
#include <iostream>

Packet::Packet()
{
	m_overhead = 0;
    m_size = 0;
	m_isAFragment=0;
	m_isTheLatestFragment = 0;
	m_num_application = -1;
	m_retransmit_number = 0;
	m_is_FountainCode = false; //Ĭ�ϲ���
  
}

Packet::~Packet()
{

}


Packet*
Packet::Copy ()
{
  Packet *p = new Packet ();
  p->m_overhead = m_overhead;
  p->m_size = m_size;
  p->m_isAFragment = m_isAFragment;
  p->m_isTheLatestFragment = m_isTheLatestFragment;
  p->m_num_application = m_num_application;
  p->m_retransmit_number = m_retransmit_number;
  p->m_is_FountainCode = m_is_FountainCode;
  return p;
}


void
Packet::Print ()
{  
	if (m_is_FountainCode)
	 std::cout << "�ð�Ϊ��Ȫ��� "<<std::endl;
	else
	std::cout << "�ð���Ϊ��Ȫ��� " << std::endl;

	std::cout <<" ******** \n **Packet: ����С: " << m_size;
	std::cout << "**��Ӧ����ҵ�����ݴ�С: " << m_size;
  if   (m_isAFragment)
	   std::cout << "**ΪС����Ƭ,";
  if (m_isTheLatestFragment)
	  std::cout << "**Ϊ���С����Ƭ"<< std::endl; 

  if (m_retransmit_number)
	  std::cout << "**�ð��ش���" << m_retransmit_number << "��" << std::endl;
}
