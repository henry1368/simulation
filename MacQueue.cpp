

#include "MacQueue.h"
#include "application.h"
#include "NetworkNode.h"
#include "NetworkManager.h"
#include "simulator.h"
#include <iostream>


MacQueue::MacQueue()
{

  m_queue = new PacketQueue ();
}

MacQueue::~MacQueue()
{
  m_queue->clear ();
  delete m_queue;
}


MacQueue::PacketQueue*
MacQueue::GetPacketQueue() const              //���ض���
{
	return m_queue;
}

void
MacQueue::Enqueue(Application::flowNode *pack)
{
	QueueElement *element = new QueueElement(pack);
	GetPacketQueue()->push_back(element);         //��¼��Ϣ
}

void
MacQueue::Dequeue()
{
	GetPacketQueue()->pop_front();                //ɾ�����нڵ㣨������ɻ�������
}

bool
MacQueue::IsEmpty() const                     //�ж϶����Ƿ�Ϊ��
{
	return GetPacketQueue()->empty();
}




Packet*
MacQueue::GetPacketToTramsit(int Net_num, double availableBits)    //���������������ţ�������С
{
  QueueElement* elm = GetPacketQueue()->front();                //ȡ����һ����Ϣ�ڵ�

  if (elm->m_Retransmit_packet != NULL)
	  return elm->m_Retransmit_packet;                          //����Ҫ�ش��İ�

  else
  {
	  Packet *packet = new Packet();
	  double overhead = int(availableBits*0);                   //����
	  double dataToSend = availableBits - overhead;               //��Ч����
	  packet->m_overhead = overhead ;	            
	  packet->m_num_application = num_of_app; 

	  if (Net_num == 2)
	  {
          packet->m_is_FountainCode = true;               //��Ȫ�벻����������ΪĿ�꣬���Ǿ�������Ȫ�����֪���ܳɹ�����Ϊֹ
		  packet->m_size = dataToSend;
		  return  packet;
	  }
		                    
	
	  double  Total_size = elm->m_flowNode->m_data_size;

	  if (elm->m_fragmentation)                        //���������Ƭ�Ļ�������Ҳ��Ȼ�Ƿ�Ƭ
	  {
		  std::cout << "the packet is a fragment" << std::endl;   //�˴δ���Ϊ��Ƭ
		  packet->m_isAFragment = 1;

		  if (dataToSend < Total_size - elm->m_fragmentOffset)
		  {
			  packet->m_size = dataToSend;
			  packet->m_isTheLatestFragment = false;
		  }
		  else
		  {
			  packet->m_size = Total_size - elm->m_fragmentOffset;
			  packet->m_isTheLatestFragment = true;
		  }

		  elm->m_fragmentNumber++;                          //��Ƭ��Ŀ��1
		  elm->m_fragmentOffset += packet->m_size;          //��Ƭλ�ñ�־ m_fragmentOffset��ʾ��Ƭ�Ѵ��������λ
		  elm->m_fragmentation = true;                      //��Ƭ
	  }

	  else if (dataToSend < Total_size - elm->m_fragmentOffset)   //�Ƚ�����ʣδ�������ݴ�С
	  {
		  std::cout << "the packet is a fragment" << std::endl;   //�˴δ���Ϊ��Ƭ
		  packet->m_size = dataToSend;
		  packet->m_isAFragment = 1;
		  packet->m_isTheLatestFragment = false;      

		  elm->m_fragmentNumber++;                 
		  elm->m_fragmentOffset += packet->m_size;
		  elm->m_fragmentation = true;
	  }

	  else
	  {
		  std::cout << "the packet is not a fragment" << std::endl;   //�˴δ��䲻Ϊ��Ƭ
		  packet->m_size = Total_size;
		  packet->m_isAFragment = false;
		  packet->m_isTheLatestFragment = false;        //�޷�Ƭ
		  elm->m_fragmentation = false;

	  }
	  return packet;

  }

}



MacQueue::QueueElement::QueueElement ()
{
  m_flowNode = NULL;
  m_fragmentation = false;
  m_fragmentNumber = 0;
  m_fragmentOffset = 0;
  m_Retransmit_packet = NULL;

}

MacQueue::QueueElement::QueueElement(Application::flowNode *pack)
{
  m_flowNode = pack;
  m_fragmentation = false;
  m_fragmentNumber = 0;
  m_fragmentOffset = 0;
  m_Retransmit_packet = NULL;
}

MacQueue::QueueElement::~QueueElement()
{
	delete m_flowNode;
	delete m_Retransmit_packet;
	m_flowNode = NULL;
	m_Retransmit_packet = NULL;
}

void
MacQueue::QueueElement::Reset()
{
	m_fragmentation = false;
	m_fragmentNumber = 0;
	m_fragmentOffset = 0;
	m_Retransmit_packet = NULL;
}



void 
MacQueue::SetAppNum(int k)
{
	num_of_app = k;
}
int 
MacQueue::GetAppNum()
{
	return num_of_app;
}



void
MacQueue::PrintQueueInfo ()
{

  std::cout << "\t ** queue info: " << std::endl;

  PacketQueue *queue = GetPacketQueue ();
  PacketQueue::iterator iter;

  for (iter = queue->begin (); iter != queue->end (); iter++)
   {
	 std:: cout << "\t ** pkt --> "	<< std::endl;
   }

}
