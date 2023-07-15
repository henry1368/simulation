


#include "messages.h"
#include "NetworkNode.h"
#include "application.h"


Message::Message()
{
m_source = NULL;
m_destination = NULL;

}


Message::~Message ()
{
}


// ----------------------------------------------------------------------------------------------------------


WIFISendingMessage::WIFISendingMessage()
{
	m_UeWifiSendingMessage = new vector<struct SendingRecord>;
 
}


WIFISendingMessage::~WIFISendingMessage()
{
  m_UeWifiSendingMessage->clear();
  delete m_UeWifiSendingMessage;
}



