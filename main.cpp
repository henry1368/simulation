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
	int * x;                                                //�ṹ����Ϣ���ݲ���,����չ
	int * y;
	DWORD dwx;
}thread_para;

DWORD WINAPI handle_lte(void *a)                           //�߳�1����LTE�¼�����������Ϣ
{ 
	thread_para * ptr1 = (thread_para*)a;
	//puts("Start to receive message");
	MSG msg11;
	while (GetMessage(&msg11, NULL, 0, 0))
	{


//		cout << "11 receive_OK" << endl;
//		cout << "11 process......." << endl;
		//		*(ptr1->x) = 0;
		ptr1->pt->ProcessOneEvent();    	                //�����¼�
//		cout << "handle over!" << endl;
		*(ptr1->x) = 1;
		PostThreadMessage(ptr1->dwx, ms1, 0, 0);            //����Ϣ�����߳�


	}
	return 0;
}


DWORD WINAPI handle_wifi(void *a)                           //�߳�2����wifi�¼�����������Ϣ
{
	thread_para * ptr2 = (thread_para*)a;
	//	puts("Start to receive message");
	MSG msg22;
	while (GetMessage(&msg22, NULL, 0, 0))
	{

//		cout << "22 receive_OK" << endl;
		//			*(ptr2->y) = 0;
//		cout << "22 process......." << endl;
		ptr2->pt->ProcessOneEvent();                   //�����¼�
//		cout << "handle over!" << endl;
		*(ptr2->y) = 1;
		PostThreadMessage(ptr2->dwx, ms2, 0, 0);       //����Ϣ�����߳�
	}
	return 0;
}


void main()
{ 
    Simulator * ptr = Simulator::Init();                        //�����б��ʼ��
	Init* m_init = new Init();
	m_init->Net_Init();                             //�����ʼ�����������¼�
                      
	FrameManager *frameManager = FrameManager::Init();          //LTE-A��ʼ��             
	DcfManager* dcfManager = DcfManager::Init();                //WIFI��ʼ��

   
	ptr->Run();

//	int flag1 = 1;                                               //Ԥ�����ݲ���
//	int flag2 = 1;
//	DWORD dw0 = GetCurrentThreadId();
//	DWORD  dw1, dw2;
//	
//	thread_para PTR = { ptr, &flag1, &flag2, dw0 };
//	HANDLE hThread1 = CreateThread(NULL, 0, handle_lte, &PTR, 0, &dw1);      //����һ���߳�
//	HANDLE hThread2 = CreateThread(NULL, 0, handle_wifi, &PTR, 0, &dw2);     //����һ���߳�
//	MSG msg;
//	while (!(ptr->GetCalendar()->IsEmpty()) && !(ptr->ifstop()))
//	{
////		cout << "δ�����������" << ptr->GetunscheduledEvents() << "��" << endl;
//				Sleep(100);
//		if (strcmp(ptr->GetCalendar()->GetEvent()->GetAttribution(), "lte") == 0)
//		{
//			PostThreadMessage(dw1, ms1, 0, 0);                //�����߳�1����Ϣ
//
//			while (!GetMessage(&msg, NULL, 0, 0))             //����
//			{
//				Sleep(100);
//			}
//		}
//
//		else if (strcmp(ptr->GetCalendar()->GetEvent()->GetAttribution(), "wifi") == 0)
//		{
//
//			PostThreadMessage(dw2, ms2, 0, 0);                // �����߳�2����Ϣ
//
//			while (!GetMessage(&msg, NULL, 0, 0))             //����
//			{
//				Sleep(100);
//			}
//		}
//	}
}