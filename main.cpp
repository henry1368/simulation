#include "init.h"
#include "time.h"
#include "parameter.h"
#include"iostream"
#include "simulator.h"
#include"windows.h"
#include "string.h"
#include"FrameManager.h"
#include "dcf-manager.h"

#define ms1   WM_USER+1  
#define ms2   WM_USER+2
#define ms3   WM_USER+3
using namespace std;

typedef struct{
	Simulator * pt;
	int * x;                                                //结构体信息传递参数,可扩展
	int * y;
	DWORD dwx;
}thread_para;

DWORD WINAPI handle_lte(void *a)                           //线程1处理LTE事件结束返回消息
{ 
	thread_para * ptr1 = (thread_para*)a;
	//puts("Start to receive message");
	MSG msg11;
	while (GetMessage(&msg11, NULL, 0, 0))
	{


//		cout << "11 receive_OK" << endl;
//		cout << "11 process......." << endl;
		//		*(ptr1->x) = 0;
		ptr1->pt->ProcessOneEvent();    	                //处理事件
//		cout << "handle over!" << endl;
		*(ptr1->x) = 1;
		PostThreadMessage(ptr1->dwx, ms1, 0, 0);            //发消息给主线程


	}
	return 0;
}


DWORD WINAPI handle_wifi(void *a)                           //线程2处理wifi事件结束返回消息
{
	thread_para * ptr2 = (thread_para*)a;
	//	puts("Start to receive message");
	MSG msg22;
	while (GetMessage(&msg22, NULL, 0, 0))
	{

//		cout << "22 receive_OK" << endl;
		//			*(ptr2->y) = 0;
//		cout << "22 process......." << endl;
		ptr2->pt->ProcessOneEvent();                   //处理事件
//		cout << "handle over!" << endl;
		*(ptr2->y) = 1;
		PostThreadMessage(ptr2->dwx, ms2, 0, 0);       //发消息给主线程
	}
	return 0;
}


void main()
{ 
    Simulator * ptr = Simulator::Init();                        //引擎列表初始化
	Init* m_init = new Init();
	m_init->Net_Init();                             //网络初始化，并安排事件
                      
	FrameManager *frameManager = FrameManager::Init();          //LTE-A初始化             
	DcfManager* dcfManager = DcfManager::Init();                //WIFI初始化

   
	ptr->Run();

//	int flag1 = 1;                                               //预留传递参数
//	int flag2 = 1;
//	DWORD dw0 = GetCurrentThreadId();
//	DWORD  dw1, dw2;
//	
//	thread_para PTR = { ptr, &flag1, &flag2, dw0 };
//	HANDLE hThread1 = CreateThread(NULL, 0, handle_lte, &PTR, 0, &dw1);      //创建一个线程
//	HANDLE hThread2 = CreateThread(NULL, 0, handle_wifi, &PTR, 0, &dw2);     //创建一个线程
//	MSG msg;
//	while (!(ptr->GetCalendar()->IsEmpty()) && !(ptr->ifstop()))
//	{
////		cout << "未处理的事情有" << ptr->GetunscheduledEvents() << "件" << endl;
//				Sleep(100);
//		if (strcmp(ptr->GetCalendar()->GetEvent()->GetAttribution(), "lte") == 0)
//		{
//			PostThreadMessage(dw1, ms1, 0, 0);                //给子线程1发消息
//
//			while (!GetMessage(&msg, NULL, 0, 0))             //回馈
//			{
//				Sleep(100);
//			}
//		}
//
//		else if (strcmp(ptr->GetCalendar()->GetEvent()->GetAttribution(), "wifi") == 0)
//		{
//
//			PostThreadMessage(dw2, ms2, 0, 0);                // 给子线程2发消息
//
//			while (!GetMessage(&msg, NULL, 0, 0))             //回馈
//			{
//				Sleep(100);
//			}
//		}
//	}
}