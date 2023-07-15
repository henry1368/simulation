

#ifndef MACQUEUE_H_
#define MACQUEUE_H_

#include "Packet.h"
#include "application.h"


#include <deque>
#include <stdint.h>

using namespace std;      //每个ap和基站中存储有发送用户数据的缓存队列


class MacQueue {

public:

	struct QueueElement
	{
		QueueElement();
		QueueElement(Application::flowNode* pack);   //业务的数据流的信息
		virtual ~QueueElement();
		void Reset();


		Application::flowNode*  m_flowNode;       //该流节点
		bool m_fragmentation;                     //该流是否分片
		double m_fragmentNumber;                     //分片数目
		double m_fragmentOffset;                //分片的补偿数目

		Packet * m_Retransmit_packet;            //该流中需要重传的包  
	};
   typedef std::deque<QueueElement*> PacketQueue;      //数据缓存队列


	MacQueue();
	virtual ~MacQueue();

	PacketQueue*
	GetPacketQueue() const;                   //返回数据队列

	void
	Enqueue(Application::flowNode* pack);         //入队
	
	Packet*
	GetPacketToTramsit(int Net_num,double availableBits);       //根据可传输字节生成包;

	bool
	IsEmpty () const;                        //是否为空

	void
	Dequeue ();                                  //删除节点

	void SetAppNum(int k);
	int GetAppNum();                         //业务数



	void PrintQueueInfo ();                 //打印信息

private:

	PacketQueue *m_queue;
	int num_of_app;                        //队列对应业务
};

#endif 