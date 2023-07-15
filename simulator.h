
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
	static Simulator *ptr;       //本类的指针

	Calendar *m_calendar;       //calendar的指针
	bool m_stop;                //布尔值，是否停止
	int m_currentUid;           //当前事件id
	double m_currentTs;         //当前时间
	int m_unscheduledEvents;    //整型，未处理事件个数

	int m_uid;                  //id



public:
	void ProcessOneEvent();
	virtual ~Simulator();

	static Simulator*
		Init()
	{
			if (ptr == NULL)
			{
				ptr = new Simulator;     //初始化，如若ptr为空，则开辟类空间
			}
			return ptr;
		}

	double Now();               //当前Ts时间

	void Run();                 //运行
	void Stop();                //停止
	void SetStop(double time);      //设置停止时间

	int GetUID();

	void
		DoSchedule(char* attribution, double time, Event *event);   //参数为double 时间和事件类型指针

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
		Schedule(char* attribution, double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);       //调用模板

	void
		Schedule(char* attribution, double time, void(*f) ());        //调用函数

	template <typename U1>
	void
		Schedule(char* attribution, double time, void(*f) (U1));         //调用含参数函数

	template <typename U1, typename T1>
	void
		Schedule(char* attribution, double time, void(*f) (U1), T1 a1);   //调用

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


