

#ifndef _MESSAGES_H
#define _MESSAGES_H

#include <vector>
#include "application.h"
class NetworkNode;


/*
�������û����͵�AP����Ϣ���� ;�û����վ��AP��������Ϣ�Ķ���
 */
class Message
{
public:

  Message ();
  virtual ~Message ();
  NetworkNode* m_source;
  NetworkNode* m_destination;
};

#endif 



// ----------------------------------------------------------------------------------------------------------


#ifndef _WIFI_SENDING_MESSAGES_H
#define _WIFI_SENDING_MESSAGES_H

#include <list>
#include "application.h"
using namespace std;
/*
 �û���AP���͵�����Ϊ��Ϣ���ƣ��������ߡ�AP�Լ������������Ϣ
 */
class WIFISendingMessage : public Message
{
public:

	WIFISendingMessage();
	virtual ~WIFISendingMessage();

  struct SendingRecord
  {
    int m_Numofapplist;                           //�ڼ���ҵ��
	Application::flowNode * m_flowNode;           //ʲô����
  };

  vector<struct SendingRecord>*  m_UeWifiSendingMessage;

};

#endif 






