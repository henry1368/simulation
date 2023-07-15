

#ifndef MAKE_EVENT_H
#define MAKE_EVENT_H


class Event;

template <typename MEM, typename OBJ>
Event* MakeEvent (MEM mem_ptr, OBJ obj);

template <typename MEM, typename OBJ,typename T1>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1);

template <typename MEM, typename OBJ,typename T1, typename T2>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

template <typename MEM, typename OBJ,typename T1, typename T2, typename T3>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

Event* MakeEvent (void (*f) ());

template <typename U1,typename T1>
Event* MakeEvent (void (*f) (U1), T1 a1);




/********************************************************************
   Implementation of templates defined above
 ********************************************************************/

#include "event.h"

template <typename T>
struct EventMemberImplObjTraits;

template <typename T>
struct EventMemberImplObjTraits<T *> {
    static T& GetReference (T *p)          //&GetReference (T *p)可以取得成员函数的地址，所以返回是一个对象
	{
		return *p;
	}
}; 

/****************************************************************************/
/****************************************************************************/
template <typename MEM, typename OBJ>
Event* MakeEvent (MEM mem_ptr, OBJ obj)        //这里的OBJ应该是对象的指针，MEM是成员函数指针
{
 // zero argument version
	class EventMemberImpl0 : public Event                 // EventMemberImpl0是Event类的公有式派生
	{
	private:
		virtual void RunEvent()
		{
			(EventMemberImplObjTraits<OBJ>::GetReference(m_obj).*m_function)();  //成员函数指针调用使用.*或者->*   ????????
		}
		OBJ m_obj;             
		MEM m_function;     //函数指针
	public:
	EventMemberImpl0 (OBJ obj, MEM function) 
		: m_obj (obj), 
		m_function (function)                                //构造函数
	{}
	virtual ~EventMemberImpl0(){};
	}

	* ev = new EventMemberImpl0(obj, mem_ptr);
	return ev;
}
/****************************************************************************/
/****************************************************************************/

template <typename MEM, typename OBJ, typename T1>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1)
{
  // one argument version
  class EventMemberImpl1 : public Event {
  public:
    EventMemberImpl1 (OBJ obj, MEM function, T1 a1) 
      : m_obj (obj), 
        m_function (function),
        m_a1 (a1)
    {}
  protected:
    virtual ~EventMemberImpl1 () {}
  private:
    virtual void RunEvent () {
      (EventMemberImplObjTraits<OBJ>::GetReference (m_obj).*m_function) (m_a1);
    }
    OBJ m_obj;                //对象
    MEM m_function;
    T1 m_a1;
  }
  *ev = new EventMemberImpl1 (obj, mem_ptr, a1);
  return ev;
}

template <typename MEM, typename OBJ,typename T1, typename T2>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1, T2 a2)
{
  // two argument version
  class EventMemberImpl1b : public Event {
  public:
    EventMemberImpl1b (OBJ obj, MEM function, T1 a1, T2 a2)
      : m_obj (obj),
        m_function (function),
        m_a1 (a1),
        m_a2 (a2)
    {}
  protected:
    virtual ~EventMemberImpl1b () {}
  private:
    virtual void RunEvent () {
      (EventMemberImplObjTraits<OBJ>::GetReference (m_obj).*m_function) (m_a1, m_a2);
    }
    OBJ m_obj;
    MEM m_function;
    T1 m_a1;
    T2 m_a2;
  }
  *ev = new EventMemberImpl1b (obj, mem_ptr, a1, a2);
  return ev;                                             //返回
}


template <typename MEM, typename OBJ, typename T1, typename T2, typename T3>
Event* MakeEvent (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3)
{
  // two argument version
  class EventMemberImpl1b : public Event {
  public:
    EventMemberImpl1b (OBJ obj, MEM function, T1 a1, T2 a2, T3 a3)          //构造函数
      : m_obj (obj),
        m_function (function),
        m_a1 (a1),
        m_a2 (a2),
        m_a3 (a3)
    {}
  protected:
    virtual ~EventMemberImpl1b () {}
  private:
    virtual void RunEvent () {
      (EventMemberImplObjTraits<OBJ>::GetReference (m_obj).*m_function) (m_a1, m_a2, m_a3);
    }
    OBJ m_obj;
    MEM m_function;                
    T1 m_a1;
    T2 m_a2;
    T3 m_a3;
  }
  *ev = new EventMemberImpl1b (obj, mem_ptr, a1, a2, a3);
  return ev;
}
/*
Event*
MakeEvent (void (*f) ())
{
  // zero arg version
  class EventFunctionImpl0 : public EventImpl
  {
  public:
    typedef void (*F)();

    EventFunctionImpl0 (F function)
      : m_function (function)
    {}
    virtual ~EventFunctionImpl0 () {}
  protected:
    virtual void RunEvent () {
      (*m_function) ();
    }
  private:
  	F m_function;
  }
  *ev = new EventFunctionImpl0 (f);
  return ev;
}
*/

template <typename U1, typename T1>
Event* MakeEvent (void (*f) (U1), T1 a1)
{
  // one arg version
  class EventFunctionImpl2 : public Event {
  public:
	  typedef void(*F)(U1);                 //?????????
      
    EventFunctionImpl2 (F function, T1 a1)
      : m_function (function),                 //构造函数
          m_a1(a1)                                 //？？？？缺少m_a1?
    { }
  protected:
    virtual ~EventFunctionImpl2 () {}
  private:
    virtual void RunEvent () {
      (*m_function) (m_a1);
    }
    F m_function;     //F类型函数指针
    T1 m_a1;          
  }
  *ev = new EventFunctionImpl2 (f, a1);
  return ev;
}


#endif /* MAKE_EVENT_H */
