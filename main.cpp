#include "init.h"
#include "time.h"
#include "parameter.h"
#include"iostream"
#include "simulator.h"
#include "string.h"
#include"FrameManager.h"
#include "dcf-manager.h"

using namespace std;




int main()
{ 
    Simulator * ptr = Simulator::Init();                        //�����б��ʼ��
	Init* m_init = new Init();
	m_init->Net_Init();                             //�����ʼ�����������¼�
                      
	FrameManager *frameManager = FrameManager::Init();          //LTE-A��ʼ��             
	DcfManager* dcfManager = DcfManager::Init();                //WIFI��ʼ��

   
	ptr->Run();


    return 0;
}
