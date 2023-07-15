



#ifndef CALENDAR_H_
#define CALENDAR_H_

#include "event.h"
#include <list>       //包含链表的函数


class Calendar {
public:
	Calendar();
	virtual ~Calendar();

    typedef std::list<Event*> Events;    //一个存储Event指针的双向链表容器，定义为Events

    Events*
    GetEvents ();           //返回当前时间指针

	void
	InsertEvent (Event *newEvent);        
	bool 
	IsEmpty ();             //是否为空
	Event*
	GetEvent ();          //第一个事件
	void
	RemoveEvent ();         //移除第一个事件


private:
    Events *m_events;          //事件链表的指针
};

#endif 
