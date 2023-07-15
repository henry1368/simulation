


#include "calendar.h"

#include <iostream>

Calendar::Calendar()
{
  m_events = new Events;          //���캯��
}

Calendar::~Calendar()
{
  delete m_events;                //�۹�����
}

Calendar::Events*
Calendar::GetEvents ()        //���������¼������ָ��
{ 
  return m_events;            
}

void
Calendar::InsertEvent (Event *newEvent)
{
  Events *events = GetEvents ();           //�¼�����
  Event *event;                            //ͷ�¼�
  Events::iterator iter;                   //����������������������

  if (IsEmpty ())                     
    {
	  events->push_front(newEvent);         //���Ϊ�ղ����¼�
	  return;
    }

  for (iter = events->begin(); iter != events->end(); iter++)
	{
	  event = *iter;
	  if(newEvent->GetTimeStamp() < event->GetTimeStamp())         //��ʱ��˳������¼�
	    {
		  m_events->insert(iter, newEvent); 
		  return;
	    }
	}

  m_events->push_back(newEvent);;
}

bool
Calendar::IsEmpty ()							   //�Ƿ�Ϊ��
{
  return GetEvents ()->empty();           
}

Event*
Calendar::GetEvent ()					          //���ص�ǰʱ������ĵ�һ���¼�
{
  if (IsEmpty ())
	return NULL;

  Event *event = GetEvents ()->front ();      
  return event;
}

void
Calendar::RemoveEvent ()                         //ɾ����ǰ�ĵ�һ���¼������¼��Ѿ������
{
  if (!IsEmpty ())
    {
	  Event *event = GetEvents ()->front(); 
	  GetEvents()->pop_front();                   //�����趨����ĵ�һ���¼�
	  delete event;
    }
}


