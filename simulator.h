
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "event.h"
#include "calendar.h"
#include "make-event.h"

#include <stdint.h>
#include <string>
#include <iostream>


/*
* Simulator
* Control the scheduling of simulation events.
*/

class Simulator
{
private:
	Simulator();
	static Simulator *ptr;       //�����ָ��

	Calendar *m_calendar;       //calendar��ָ��
	bool m_stop;                //����ֵ���Ƿ�ֹͣ
	int m_currentUid;           //��ǰ�¼�id
	double m_currentTs;         //��ǰʱ��
	int m_unscheduledEvents;    //���ͣ�δ�����¼�����

	int m_uid;                  //id



public:
	void ProcessOneEvent();
	virtual ~Simulator();

	static Simulator*
		Init()
	{
			if (ptr == NULL)
			{
				ptr = new Simulator;     //��ʼ��������ptrΪ�գ��򿪱���ռ�
			}
			return ptr;
		}

	double Now();               //��ǰTsʱ��

	void Run();                 //����
	void Stop();                //ֹͣ
	void SetStop(double time);      //����ֹͣʱ��

	int GetUID();

	void
		DoSchedule(char* attribution, double time, Event *event);   //����Ϊdouble ʱ����¼�����ָ��

	Calendar * GetCalendar();
	bool ifstop();
	int GetunscheduledEvents();

	/*
	* Schedule methods are called to insert a new method
	* into the calendar scheduler
	*/
	template <typename MEM, typename OBJ>
	void
		Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj);

	template <typename MEM, typename OBJ, typename T1>
	void
		Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1);

	template <typename MEM, typename OBJ, typename T1, typename T2>
	void
		Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

	template <typename MEM, typename OBJ, typename T1, typename T2, typename T3>
	void
		Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);       //����ģ��

	void
		Schedule(char* attribution, double time, void(*f) ());        //���ú���

	template <typename U1>
	void
		Schedule(char* attribution, double time, void(*f) (U1));         //���ú���������

	template <typename U1, typename T1>
	void
		Schedule(char* attribution, double time, void(*f) (U1), T1 a1);   //����

	void
		PrintMemoryUsage();
};


template <typename MEM, typename OBJ>
void
Simulator::Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj)
{
	DoSchedule(attribution, time, MakeEvent(mem_ptr, obj));
}

template <typename MEM, typename OBJ, typename T1>
void
Simulator::Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1)
{
	DoSchedule(attribution, time, MakeEvent(mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ, typename T1, typename T2>
void
Simulator::Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2)
{
	DoSchedule(attribution, time, MakeEvent(mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ, typename T1, typename T2, typename T3>
void
Simulator::Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3)
{
	DoSchedule(attribution, time, MakeEvent(mem_ptr, obj, a1, a2, a3));
}

/*
void
Simulator::Schedule (double time, void (*f) ())
{
//NON FUNZIONA ANCORA !?!?!?!
//DoSchedule (time, MakeEvent (f));
}
*/

template <typename U1, typename T1>
void
Simulator::Schedule(char* attribution, double time, void(*f) (U1), T1 a1)
{
	DoSchedule(attribution, time, MakeEvent(f, a1));
}



#endif /* SIMULATOR_H */


