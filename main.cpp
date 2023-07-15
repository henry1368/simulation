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
    Simulator * ptr = Simulator::Init();                        //引擎列表初始化
	Init* m_init = new Init();
	m_init->Net_Init();                             //网络初始化，并安排事件
                      
	FrameManager *frameManager = FrameManager::Init();          //LTE-A初始化             
	DcfManager* dcfManager = DcfManager::Init();                //WIFI初始化

   
	ptr->Run();


    return 0;
}
