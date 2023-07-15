
#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_

#include <iostream>
#include "simulator.h"



class FrameManager {
public:

 //Ĭ��֡�ṹ��FDD

private:

	unsigned long m_TTICounter;               //TTI����
	FrameManager();							  //���캯��
	static FrameManager *ptr;				  //ָ��

public:
	//FrameManager();
	virtual ~FrameManager();

	static FrameManager*
	Init ()
	  { 
		if (ptr==NULL)
	      {
		    ptr = new FrameManager;          //��ʼ����ͬ��ptr = new FrameManager����
	   	  }
		return ptr;
	  }


	void UpdateTTIcounter ();                             //TTI+1
	unsigned long GetTTICounter () const;                 //��ȡTTI

	void Start ();

	void StartSubframe ();                               //TTI��ʼ

	void StopSubframe ();                                //TTI����




};

#endif 
