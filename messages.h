

#ifndef _MESSAGES_H
#define _MESSAGES_H

#include <vector>
#include "application.h"
class NetworkNode;


/*
包括从用户发送到AP的信息定义 ;用户向基站、AP反馈等消息的定义
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
 用户给AP发送的数据为消息机制，即“告诉”AP自己所需的数据信息
 */
class WIFISendingMessage : public Message
{
public:

	WIFISendingMessage();
	virtual ~WIFISendingMessage();

  struct SendingRecord
  {
    int m_Numofapplist;                           //第几个业务
	Application::flowNode * m_flowNode;           //什么数据
  };

  vector<struct SendingRecord>*  m_UeWifiSendingMessage;

};

#endif 






