
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
	m_is_FountainCode = false; //默认不是
  
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
	 std::cout << "该包为喷泉码包 "<<std::endl;
	else
	std::cout << "该包不为喷泉码包 " << std::endl;

	std::cout <<" ******** \n **Packet: 包大小: " << m_size;
	std::cout << "**对应所在业务数据大小: " << m_size;
  if   (m_isAFragment)
	   std::cout << "**为小包分片,";
  if (m_isTheLatestFragment)
	  std::cout << "**为最后小包分片"<< std::endl; 

  if (m_retransmit_number)
	  std::cout << "**该包重传了" << m_retransmit_number << "次" << std::endl;
}
