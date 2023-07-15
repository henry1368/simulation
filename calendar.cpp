


#include "calendar.h"

#include <iostream>

Calendar::Calendar()
{
  m_events = new Events;          //构造函数
}

Calendar::~Calendar()
{
  delete m_events;                //折构函数
}

Calendar::Events*
Calendar::GetEvents ()        //返回所在事件链表的指针
{ 
  return m_events;            
}

void
Calendar::InsertEvent (Event *newEvent)
{
  Events *events = GetEvents ();           //事件链表
  Event *event;                            //头事件
  Events::iterator iter;                   //？？？？？？？？？？？

  if (IsEmpty ())                     
    {
	  events->push_front(newEvent);         //如果为空插入事件
	  return;
    }

  for (iter = events->begin(); iter != events->end(); iter++)
	{
	  event = *iter;
	  if(newEvent->GetTimeStamp() < event->GetTimeStamp())         //按时间顺序插入事件
	    {
		  m_events->insert(iter, newEvent); 
		  return;
	    }
	}

  m_events->push_back(newEvent);;
}

bool
Calendar::IsEmpty ()							   //是否为空
{
  return GetEvents ()->empty();           
}

Event*
Calendar::GetEvent ()					          //返回当前时间链表的第一个事件
{
  if (IsEmpty ())
	return NULL;

  Event *event = GetEvents ()->front ();      
  return event;
}

void
Calendar::RemoveEvent ()                         //删除当前的第一个事件（该事件已经解决）
{
  if (!IsEmpty ())
    {
	  Event *event = GetEvents ()->front(); 
	  GetEvents()->pop_front();                   //重新设定链表的第一个事件
	  delete event;
    }
}


