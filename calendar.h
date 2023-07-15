



#ifndef CALENDAR_H_
#define CALENDAR_H_

#include "event.h"
#include <list>       //��������ĺ���


class Calendar {
public:
	Calendar();
	virtual ~Calendar();

    typedef std::list<Event*> Events;    //һ���洢Eventָ���˫����������������ΪEvents

    Events*
    GetEvents ();           //���ص�ǰʱ��ָ��

	void
	InsertEvent (Event *newEvent);        
	bool 
	IsEmpty ();             //�Ƿ�Ϊ��
	Event*
	GetEvent ();          //��һ���¼�
	void
	RemoveEvent ();         //�Ƴ���һ���¼�


private:
    Events *m_events;          //�¼������ָ��
};

#endif 
