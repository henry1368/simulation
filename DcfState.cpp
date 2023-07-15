#include "DcfState.h"
#include "stdlib.h"
#include "application.h"
#include "NetworkNode.h"
#include "AP.h"
#include "UserEquipment.h"
#include "simulator.h"
#include "time.h"

DcfState::DcfState()
{

   //初始化时，赋值均为0，后面根据业务模型进行重新设置
	m_aifsn = 0;                      //和业务类型有关;            
    m_backoffSlots=0;                 //剩余退避时间
	m_totalbackofftime = 0;            //总退避时间
	m_cw = 0;                          
	m_SRC = 0;                        //初始化设定SRC和LRC均为0
	m_LRC = 0;
	m_accessRequested = 0;
	m_accesstime = 0;                 //何时接入也和业务类型有关;  
	m_backoffStart = 0;	              //退避开始时间
	Is_initial = false;               //标志位，是否已经初始化
};


DcfState::~DcfState()
{
}




void
DcfState::ResetState()
{
	/*
	用户在初始化退避状态时、发包成功重新确定下一个退避状态时，或者是进行基站的切换后根据业务模型确定新的退避状态
	根据业务类型确定各参数的值。
	*/

	//如果wifi网络同时存在多个业务，则找到最早时间的业务赋值（该业务信息未发送，即AP不知给用户所发数据）
	if (Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_UE)
	{
		vector<Application*>* m_applist = ((UserEquipment*)Belong_Node)->GetApplist();   

		double t = 10e10;
		int index = -1;

		for (unsigned int i = 0; i < m_applist->size(); i++)              //如果业务中数据已经全部发送，则会将该业务删除
		{
			if (m_applist->at(i)->m_Num_of_net == 1 ||
				(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))   //选出wifi网络业务  (fullbuffer业务除外)
			{
				list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
				list<Application::flowNode *>::iterator it;
				for (it = m_FL->begin(); it != m_FL->end(); it++)      //未发送业务信息的最早时间
				{
					if ((*it)->m_ue_send == false)
					{
						if ((*it)->m_data_arrive_time < t)
						{
							t = (*it)->m_data_arrive_time;
							index = i;
						}
						break;
					}
				}
			}
		}

				
	  if (index != -1)                                        //inde不为-1表示的是时间顺序最靠前的业务索引
	   {
		
		switch (m_applist->at(index)->GetApptype())                      //根据业务情况
		{
			
               case Application::ApplicationType::TYPE_VIDEO:
		         {
							m_cw = VI_CwMin;
							m_cw_max = VI_CwMax;
							m_aifsn = SIFS + SlotTime*VI_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
						
	                        m_backoffSlots = rand() % m_cw;            //退避数从（0,m_cw）中随机产生   设定随机退避时隙
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
							m_accesstime = t;         //接入时间为业务到达时间
							Is_initial = true;
							break;

			   }
				  

			   case Application::ApplicationType::TYPE_VOIP:
			   {
							m_cw = VO_CwMin;
						    m_cw_max = VO_CwMax;
							m_aifsn = SIFS + SlotTime*VO_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
					
							m_backoffSlots = rand() % m_cw;            //退避数从（0,m_cw）中随机产生   设定随机退避时隙
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
							m_accesstime = t;         //接入时间为业务到达时间
							Is_initial = true;
							break;
			   }
			
			   case Application::ApplicationType::TYPE_WEB: case Application::ApplicationType::TYPE_FTP:
			   {
							m_cw = BE_CwMin;
							m_cw_max = BE_CwMax;
							m_aifsn = SIFS + SlotTime*BE_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
					
							m_backoffSlots = rand() % m_cw;            //退避数从（0,m_cw）中随机产生   设定随机退避时隙
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
							m_accesstime = t;         //接入时间为业务到达时间
							Is_initial = true;
							break;
			   }
               
			   case Application::ApplicationType::TYPE_GAME:
			   {
						   m_cw = BK_CwMin;
						   m_cw_max = BK_CwMax;
						   m_aifsn = SIFS + SlotTime*BK_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
						
						   m_backoffSlots = rand() % m_cw;            //退避数从（0,m_cw）中随机产生   设定随机退避时隙
						   m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
						   m_accesstime = t;         //接入时间为业务到达时间
						   Is_initial = true;
						   break;
			   }
              	            
		   }
		}
	}
	else if (Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_AP)              //AP的参考设置
	{
	          
		//对AP进行设置，由于AP对多个用户同时进行传输多种业务，因此参数的确定没有参考准则，暂定为按照

		m_cw = VO_CwMin;    //   VO_CwMin;
		m_cw_max = VO_CwMax;
		m_aifsn = SIFS + SlotTime*VO_AIFSN;// VO_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
	
//		srand((unsigned int)time(NULL));
		m_backoffSlots = rand() % m_cw;            //退避数从（0,m_cw）中随机产生   设定随机退避时隙
		m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
//		cout << m_totalbackofftime << endl;
		m_accesstime = Simulator::Init()->Now();   //对于AP而言，只要缓存队列中有数据，则AP就会去竞争接入
		Is_initial = true;
	
	}


}

void
DcfState::UpdateFailedState()
{
 //传输失败时会导致退避窗口m_cw翻倍，但不超过上限

	m_cw = min(2 * (m_cw + 1) - 1, m_cw_max);   //退避区间翻倍
//	srand((unsigned int)time(NULL));
	m_backoffSlots = rand() % m_cw;             //退避数从（0,m_cw）中随机产生
	m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //退避总时间
}



bool
DcfState::IsAccessRequested() const
{
	return m_accessRequested;               //查看该用户是否有竞争接入需求
}

void
DcfState::NotifyAccessRequested()       //通告该用户接入竞争
{
	m_accessRequested = true;
}






