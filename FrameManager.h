
#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_

#include <iostream>
#include "simulator.h"



class FrameManager {
public:

 //默认帧结构是FDD

private:

	unsigned long m_TTICounter;               //TTI个数
	FrameManager();							  //构造函数
	static FrameManager *ptr;				  //指针

public:
	//FrameManager();
	virtual ~FrameManager();

	static FrameManager*
	Init ()
	  { 
		if (ptr==NULL)
	      {
		    ptr = new FrameManager;          //初始化等同于ptr = new FrameManager（）
	   	  }
		return ptr;
	  }


	void UpdateTTIcounter ();                             //TTI+1
	unsigned long GetTTICounter () const;                 //获取TTI

	void Start ();

	void StartSubframe ();                               //TTI开始

	void StopSubframe ();                                //TTI结束




};

#endif 
