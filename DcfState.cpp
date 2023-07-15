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

   //��ʼ��ʱ����ֵ��Ϊ0���������ҵ��ģ�ͽ�����������
	m_aifsn = 0;                      //��ҵ�������й�;            
    m_backoffSlots=0;                 //ʣ���˱�ʱ��
	m_totalbackofftime = 0;            //���˱�ʱ��
	m_cw = 0;                          
	m_SRC = 0;                        //��ʼ���趨SRC��LRC��Ϊ0
	m_LRC = 0;
	m_accessRequested = 0;
	m_accesstime = 0;                 //��ʱ����Ҳ��ҵ�������й�;  
	m_backoffStart = 0;	              //�˱ܿ�ʼʱ��
	Is_initial = false;               //��־λ���Ƿ��Ѿ���ʼ��
};


DcfState::~DcfState()
{
}




void
DcfState::ResetState()
{
	/*
	�û��ڳ�ʼ���˱�״̬ʱ�������ɹ�����ȷ����һ���˱�״̬ʱ�������ǽ��л�վ���л������ҵ��ģ��ȷ���µ��˱�״̬
	����ҵ������ȷ����������ֵ��
	*/

	//���wifi����ͬʱ���ڶ��ҵ�����ҵ�����ʱ���ҵ��ֵ����ҵ����Ϣδ���ͣ���AP��֪���û��������ݣ�
	if (Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_UE)
	{
		vector<Application*>* m_applist = ((UserEquipment*)Belong_Node)->GetApplist();   

		double t = 10e10;
		int index = -1;

		for (unsigned int i = 0; i < m_applist->size(); i++)              //���ҵ���������Ѿ�ȫ�����ͣ���Ὣ��ҵ��ɾ��
		{
			if (m_applist->at(i)->m_Num_of_net == 1 ||
				(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))   //ѡ��wifi����ҵ��  (fullbufferҵ�����)
			{
				list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
				list<Application::flowNode *>::iterator it;
				for (it = m_FL->begin(); it != m_FL->end(); it++)      //δ����ҵ����Ϣ������ʱ��
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

				
	  if (index != -1)                                        //inde��Ϊ-1��ʾ����ʱ��˳���ǰ��ҵ������
	   {
		
		switch (m_applist->at(index)->GetApptype())                      //����ҵ�����
		{
			
               case Application::ApplicationType::TYPE_VIDEO:
		         {
							m_cw = VI_CwMin;
							m_cw_max = VI_CwMax;
							m_aifsn = SIFS + SlotTime*VI_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
						
	                        m_backoffSlots = rand() % m_cw;            //�˱����ӣ�0,m_cw�����������   �趨����˱�ʱ϶
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
							m_accesstime = t;         //����ʱ��Ϊҵ�񵽴�ʱ��
							Is_initial = true;
							break;

			   }
				  

			   case Application::ApplicationType::TYPE_VOIP:
			   {
							m_cw = VO_CwMin;
						    m_cw_max = VO_CwMax;
							m_aifsn = SIFS + SlotTime*VO_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
					
							m_backoffSlots = rand() % m_cw;            //�˱����ӣ�0,m_cw�����������   �趨����˱�ʱ϶
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
							m_accesstime = t;         //����ʱ��Ϊҵ�񵽴�ʱ��
							Is_initial = true;
							break;
			   }
			
			   case Application::ApplicationType::TYPE_WEB: case Application::ApplicationType::TYPE_FTP:
			   {
							m_cw = BE_CwMin;
							m_cw_max = BE_CwMax;
							m_aifsn = SIFS + SlotTime*BE_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
					
							m_backoffSlots = rand() % m_cw;            //�˱����ӣ�0,m_cw�����������   �趨����˱�ʱ϶
							m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
							m_accesstime = t;         //����ʱ��Ϊҵ�񵽴�ʱ��
							Is_initial = true;
							break;
			   }
               
			   case Application::ApplicationType::TYPE_GAME:
			   {
						   m_cw = BK_CwMin;
						   m_cw_max = BK_CwMax;
						   m_aifsn = SIFS + SlotTime*BK_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
						
						   m_backoffSlots = rand() % m_cw;            //�˱����ӣ�0,m_cw�����������   �趨����˱�ʱ϶
						   m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
						   m_accesstime = t;         //����ʱ��Ϊҵ�񵽴�ʱ��
						   Is_initial = true;
						   break;
			   }
              	            
		   }
		}
	}
	else if (Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_AP)              //AP�Ĳο�����
	{
	          
		//��AP�������ã�����AP�Զ���û�ͬʱ���д������ҵ����˲�����ȷ��û�вο�׼���ݶ�Ϊ����

		m_cw = VO_CwMin;    //   VO_CwMin;
		m_cw_max = VO_CwMax;
		m_aifsn = SIFS + SlotTime*VO_AIFSN;// VO_AIFSN;         //AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime 
	
//		srand((unsigned int)time(NULL));
		m_backoffSlots = rand() % m_cw;            //�˱����ӣ�0,m_cw�����������   �趨����˱�ʱ϶
		m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
//		cout << m_totalbackofftime << endl;
		m_accesstime = Simulator::Init()->Now();   //����AP���ԣ�ֻҪ��������������ݣ���AP�ͻ�ȥ��������
		Is_initial = true;
	
	}


}

void
DcfState::UpdateFailedState()
{
 //����ʧ��ʱ�ᵼ���˱ܴ���m_cw������������������

	m_cw = min(2 * (m_cw + 1) - 1, m_cw_max);   //�˱����䷭��
//	srand((unsigned int)time(NULL));
	m_backoffSlots = rand() % m_cw;             //�˱����ӣ�0,m_cw�����������
	m_totalbackofftime = m_aifsn + m_backoffSlots*SlotTime;    //�˱���ʱ��
}



bool
DcfState::IsAccessRequested() const
{
	return m_accessRequested;               //�鿴���û��Ƿ��о�����������
}

void
DcfState::NotifyAccessRequested()       //ͨ����û����뾺��
{
	m_accessRequested = true;
}






