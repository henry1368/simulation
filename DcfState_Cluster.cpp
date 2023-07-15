#include "dcf-manager.h"
#include "DcfState_Cluster.h"
#include "DcfState.h"
#include "NetworkNode.h"
#include "AP.h"
#include "ap-mac.h"
#include "UserEquipment.h"
#include "ue-mac.h"
#include "simulator.h"
#include "packet-burst.h"
#include "messages.h"
#include "application.h"
//#include "time.h"

DcfState_Cluster::DcfState_Cluster(AP*ap)
{
	  m_state = new vector<DcfState*>;
	  m_lastNavStart=0.0;
	  m_lastNavDuration = 0.0;
	  m_TxStart = 0.0;
	  m_TxDuration = 0.0;
	  m_RxStart = 0.0;
	  m_RxDuration = 0.0;
	  m_RxEnd = 0.0;
	  m_lastRxReceiveOk = 0;
	  local_ap = ap;
}


DcfState_Cluster::~DcfState_Cluster()

{
/*
	vector<DcfState*>::iterator it;
	for (vector<DcfState*>::iterator it = m_state->begin(); it != m_state->end(); it++)       
		delete (*it);
*/
	m_state->clear();
}

void
DcfState_Cluster::StartCop()
{

//	cout << "AP" << local_ap->GetIDNetworkNode() << "号所在的簇在时刻 t=" <<Simulator::Init()->Now()<< " 开始竞争："<<endl;

	update_cluster_member();                                     //更新竞争列表
	NetworkNode* Tx_Node = Compete();                    //退避时隙数最小的用户进行数据的发送
	if (Tx_Node == NULL)
	{
//		cout << "HELLO　World" << endl;
		Simulator::Init()->Schedule("wifi", 224, &DcfState_Cluster::StartCop, this); //如果当前没有发生数据发送,则每隔一个时隙再进行检测
	}
	else
	{
		double time=0;

         if (Tx_Node->GetNodeType() == NetworkNode::TYPE_UE)
	 {	
			 cout << "用户" << Tx_Node->GetIDNetworkNode() << "号获得信道" << endl;
		//生成对应的消息
		WIFISendingMessage * s = new WIFISendingMessage();
		s->m_source = Tx_Node;
		s->m_destination = ((UserEquipment*)Tx_Node)->GetTargetAPNode();                           //设置源、目的节点

		vector<Application*>* m_applist = ((UserEquipment*)Tx_Node)->GetApplist();                  //根据业务情况来生成对应的消息

		double t = Simulator::Init()->Now();                                                        //当前时间
		double time = ((UserEquipment*)Tx_Node)->GetUeMac()->GetDcfstate()->m_totalbackofftime;       //该用户到退避结束

		for (unsigned int i = 0; i < m_applist->size(); i++)
		{
			if (m_applist->at(i)->m_Num_of_net == 1 ||
				(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))    //选出wifi网络业务且非FULLBUFFER业务
			{

				list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
				list<Application::flowNode *>::iterator it;
				for (it = m_FL->begin(); it != m_FL->end(); it++)
				{
					if ((*it)->m_data_arrive_time <= t + time && (*it)->m_ue_send==false)         //遍历所有的WIFI业务（fullbuffer除外）,当前时间以前的未发送数据信息 以消息机制发送
					{
						struct WIFISendingMessage::SendingRecord  R = { i, (*it) };
						s->m_UeWifiSendingMessage->push_back(R);
						(*it)->m_ue_send =true;                                       //该数据业务信息写入消息，并标记已经传输
					}
					else if ((*it)->m_data_arrive_time > t + time && (*it)->m_ue_send == false)
					{
						break;          //当业务包数目很大时，可以减少遍历的时间
					}

				}
			}
		}	   
		Simulator::Init()->Schedule("wifi", time+120, &UserEquipment::Wifi_SendPacketBurst, (UserEquipment*)Tx_Node, s);   //用户发送数据

		cout << "更新退避" << endl;
		State::iterator it;
		for (it = m_state->begin(); it != m_state->end(); it++)
		{
			if ((*it)->m_totalbackofftime == time)
				(*it)->UpdateFailedState();               //如果总的退避时刻一样则发生碰撞，优先级低的重新设置退避(默认相同退避时间中后加入的优先级高)
			else
			{
               ((*it)->m_totalbackofftime) -= time;                                      //更新其它用户的退避信息
			   cout << ((*it)->m_totalbackofftime) << endl;
			}
				
		}
		((UserEquipment*)Tx_Node)->GetUeMac()->GetDcfstate()->Is_initial = false;     //发送信息后，退避状态初始化
	 }

		 else if (Tx_Node->GetNodeType() == NetworkNode::TYPE_AP)
		 {
//			 cout << "AP" << Tx_Node->GetIDNetworkNode() << "号获得信道" << endl;
			 double time = ((AP*)Tx_Node)->GetApMac()->GetDcfstate()->m_totalbackofftime;       //AP退避结束时长

			 DcfManager::Init()->GetCluster(local_ap)->m_lastNavStart = Simulator::Init()->Now() + time;  //信道处于"忙"状态开始时间为退避结束时刻

			 Simulator::Init()->Schedule("wifi", time+120, &AP::SendPacketBurst, local_ap);         //ap发送数据

			 State::iterator it;
			 for (it = m_state->begin(); it != m_state->end(); it++)
			 {
				 if ((*it)->m_totalbackofftime == time)
					 (*it)->UpdateFailedState();               //如果总的退避时刻一样则发生碰撞，优先级低的重新设置退避
				 else
					 ((*it)->m_totalbackofftime) -= time;                                      //更新其它用户的退避信息
			 }

			 ((AP*)Tx_Node)->GetApMac()->GetDcfstate()->Is_initial = false;     //发送信息后，退避状态初始化

		 }

	}

	


}


void
DcfState_Cluster::update_cluster_member()                                              //更新竞争列表，加入簇
{
//	cout << "更新竞争用户集合：" << endl;
//	srand((unsigned int)time(NULL));
	State* new_mState = new vector<DcfState*>;
	int flag = 0;
	AP::UserEquipmentRecords* records = local_ap->GetUserEquipmentRecords();
	AP::UserEquipmentRecord *record;
	vector<AP::UserEquipmentRecord*>::iterator iter;                             //取出连接该ap的每个用户记录
	for (iter = records->begin(); iter != records->end(); iter++)
	{
		record = *iter;
		DcfState* ue_dcfstate = record->GetUE()->GetUeMac()->GetDcfstate();      //用户的信息状态


		if (ue_dcfstate->Is_initial == false)         //未初始化时根据业务进行赋值
			ue_dcfstate->ResetState();                   

		if (ue_dcfstate->Is_initial == true && ue_dcfstate->m_accesstime <= Simulator::Init()->Now())     //比较接入时间和当前仿真时间,这里注意有可能该用户没有WIFI业务，故即使赋值也没有初始化
		{
			ue_dcfstate->m_accessRequested = 1;                                  //该成员参与竞争
			new_mState->push_back(ue_dcfstate);                                  //更新簇成员
		}
/*
		else
			cout << "用户" << ue_dcfstate->Belong_Node->GetIDNetworkNode() << "不参与竞争"<<endl;
*/

		if (record->GetQueue()->size() > 0)       //AP对应的缓存队列中有数据
			        flag = 1;
	}

	if (flag == 1)
	{
	DcfState* ap_dcfstate = local_ap->GetApMac()->GetDcfstate();            //ap也要加入竞争
	if (ap_dcfstate->Is_initial == false)
		 ap_dcfstate->ResetState();
	 new_mState->push_back(ap_dcfstate);
	}

//	delete m_state;                                                             //释放内存
	m_state = new_mState;                                                       //新的簇生成
}


NetworkNode* 
DcfState_Cluster::Compete()
{
//	cout << "AP" << local_ap->GetIDNetworkNode() << "号对应的簇 开始竞争";
	DcfState* Tx_dcfstate;

	if (m_state->size() == 0)           //当前没有用户或者没有数据要进行发送

		return NULL;
	else                                
	{
	vector<DcfState*>::iterator it;
	double min_totalbackofftime = 1000000;

	for (it = m_state->begin(); it != m_state->end(); it++)
	{ 
		double totalbackofftime = (*it)->m_totalbackofftime;       //退避总时间：AIFS+BACKOFFBTIME
		//if ((*it)->Belong_Node->GetNodeType() == NetworkNode::NodeType::TYPE_UE)
		//   cout << " 用户"<<(*it)->Belong_Node->GetIDNetworkNode()<<",总退避时间为" << totalbackofftime << endl;
		//else
		//	cout << " AP" << (*it)->Belong_Node->GetIDNetworkNode() << ",总退避时间为" << totalbackofftime << endl;

		if (totalbackofftime <= min_totalbackofftime)          //相同的退避时间以后进的优先
		{
			min_totalbackofftime = totalbackofftime;
			Tx_dcfstate = (*it);                   //找到对应的退避时间最小的用户（即最先退避到0）
		}

	}
	return Tx_dcfstate->Belong_Node;               //返回所属节点
	}

}


void 
DcfState_Cluster::update_cluster_info()            //更新连接AP的全体用户的位置以及进行切换
{
//	cout << "AP" << local_ap->GetIDNetworkNode() << "号所在的簇在时刻 t=" << Simulator::Init()->Now() << " 开始更新位置：" << endl;

	AP::UserEquipmentRecords* records = local_ap->GetUserEquipmentRecords();
	AP::UserEquipmentRecords *new_records = new AP::UserEquipmentRecords();
	UserEquipment* ue;
	vector<AP::UserEquipmentRecord*>::iterator iter;                             
	for (iter = records->begin(); iter != records->end(); iter++)
	{
		ue = (*iter)->GetUE();           //用户

		ue->wifi_link_update();                                    //用户更新信息
		if (ue->GetTargetAPNode() != local_ap)
		{
			cout << "用户" << ue->GetIDNetworkNode() << "号连接的AP发生了切换，从AP" << local_ap->GetIDNetworkNode() << " 转换到了" << ue->GetTargetAPNode()->GetIDNetworkNode() << endl;
			/*如果用户发生了切换，则：
			1. 删除原AP中关于用户的缓存
			2. 用户已发送但未收到的数据仍需重新发送
			*/
			delete(*iter);
			vector<Application*>* m_applist = ue->GetApplist();
			for (unsigned int i = 0; i < m_applist->size(); i++)
			{
				if (m_applist->at(i)->m_Num_of_net == 1 ||
					(m_applist->at(i)->m_Num_of_net == 2 && m_applist->at(i)->GetApptype() != Application::ApplicationType::TYPE_FULLBUFFER))    //选出wifi网络业务且非FULLBUFFER业务
				{

					list<Application::flowNode *>*  m_FL = m_applist->at(i)->GetDatalist();
					list<Application::flowNode *>::iterator it;
					for (it = m_FL->begin(); it != m_FL->end(); it++)
					{
						if ((*it)->m_ue_send == true)          //已发送但是未收到的包（如果收到会剔除掉）都需要重新进行发送
							(*it)->m_ue_send = false;
						else
							break;                             //则后面的必然全部没有发送

					}
				}
			}

			ue->GetUeMac()->GetDcfstate()->Is_initial = false;   //新的连接AP下重新设置用户退避状态


		}

		else
			new_records->push_back(*iter);             //如果没有发生切换则
	}

	delete records;
	local_ap->SetUserEquipmentRecords(new_records);                             //重新设定
	Simulator::Init()->Schedule("wifi", 0.0, &DcfState_Cluster::StartCop, this);

}

void 
DcfState_Cluster::EndDCF()
{
	cout << "AP" << local_ap->GetIDNetworkNode() << "号所在的簇结束传输在时刻：" << Simulator::Init()->Now() << endl;
	Simulator::Init()->Schedule("wifi", 0.0, &DcfState_Cluster::StartCop, this);   
}
