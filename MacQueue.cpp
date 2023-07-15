

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
MacQueue::GetPacketQueue() const              //返回队列
{
	return m_queue;
}

void
MacQueue::Enqueue(Application::flowNode *pack)
{
	QueueElement *element = new QueueElement(pack);
	GetPacketQueue()->push_back(element);         //记录信息
}

void
MacQueue::Dequeue()
{
	GetPacketQueue()->pop_front();                //删除队列节点（发送完成或舍弃）
}

bool
MacQueue::IsEmpty() const                     //判断队列是否为空
{
	return GetPacketQueue()->empty();
}




Packet*
MacQueue::GetPacketToTramsit(int Net_num, double availableBits)    //输入参数：网络序号，传输块大小
{
  QueueElement* elm = GetPacketQueue()->front();                //取出第一个信息节点

  if (elm->m_Retransmit_packet != NULL)
	  return elm->m_Retransmit_packet;                          //有需要重传的包

  else
  {
	  Packet *packet = new Packet();
	  double overhead = int(availableBits*0);                   //开销
	  double dataToSend = availableBits - overhead;               //有效数据
	  packet->m_overhead = overhead ;	            
	  packet->m_num_application = num_of_app; 

	  if (Net_num == 2)
	  {
          packet->m_is_FountainCode = true;               //喷泉码不以数据总数为目标，而是尽力发喷泉码包，知道能成功解码为止
		  packet->m_size = dataToSend;
		  return  packet;
	  }
		                    
	
	  double  Total_size = elm->m_flowNode->m_data_size;

	  if (elm->m_fragmentation)                        //如果该流分片的话，本次也必然是分片
	  {
		  std::cout << "the packet is a fragment" << std::endl;   //此次传输为分片
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

		  elm->m_fragmentNumber++;                          //分片数目加1
		  elm->m_fragmentOffset += packet->m_size;          //分片位置标志 m_fragmentOffset表示分片已传输的数据位
		  elm->m_fragmentation = true;                      //分片
	  }

	  else if (dataToSend < Total_size - elm->m_fragmentOffset)   //比较与所剩未传输数据大小
	  {
		  std::cout << "the packet is a fragment" << std::endl;   //此次传输为分片
		  packet->m_size = dataToSend;
		  packet->m_isAFragment = 1;
		  packet->m_isTheLatestFragment = false;      

		  elm->m_fragmentNumber++;                 
		  elm->m_fragmentOffset += packet->m_size;
		  elm->m_fragmentation = true;
	  }

	  else
	  {
		  std::cout << "the packet is not a fragment" << std::endl;   //此次传输不为分片
		  packet->m_size = Total_size;
		  packet->m_isAFragment = false;
		  packet->m_isTheLatestFragment = false;        //无分片
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
