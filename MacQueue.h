

#ifndef MACQUEUE_H_
#define MACQUEUE_H_

#include "Packet.h"
#include "application.h"


#include <deque>
#include <stdint.h>

using namespace std;      //ÿ��ap�ͻ�վ�д洢�з����û����ݵĻ������


class MacQueue {

public:

	struct QueueElement
	{
		QueueElement();
		QueueElement(Application::flowNode* pack);   //ҵ�������������Ϣ
		virtual ~QueueElement();
		void Reset();


		Application::flowNode*  m_flowNode;       //�����ڵ�
		bool m_fragmentation;                     //�����Ƿ��Ƭ
		double m_fragmentNumber;                     //��Ƭ��Ŀ
		double m_fragmentOffset;                //��Ƭ�Ĳ�����Ŀ

		Packet * m_Retransmit_packet;            //��������Ҫ�ش��İ�  
	};
   typedef std::deque<QueueElement*> PacketQueue;      //���ݻ������


	MacQueue();
	virtual ~MacQueue();

	PacketQueue*
	GetPacketQueue() const;                   //�������ݶ���

	void
	Enqueue(Application::flowNode* pack);         //���
	
	Packet*
	GetPacketToTramsit(int Net_num,double availableBits);       //���ݿɴ����ֽ����ɰ�;

	bool
	IsEmpty () const;                        //�Ƿ�Ϊ��

	void
	Dequeue ();                                  //ɾ���ڵ�

	void SetAppNum(int k);
	int GetAppNum();                         //ҵ����



	void PrintQueueInfo ();                 //��ӡ��Ϣ

private:

	PacketQueue *m_queue;
	int num_of_app;                        //���ж�Ӧҵ��
};

#endif 