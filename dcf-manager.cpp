#include "dcf-manager.h"
#include "DcfState_Cluster.h"
#include "NetworkManager.h"
#include "AP.h"
#include "simulator.h"
using namespace std;

DcfManager* DcfManager::ptr = NULL;

DcfManager::DcfManager()                                                     //�������еĴأ�ÿһ��AP���������������û����һ����	 
{
Dcf_Cluster_Container = new std::vector<DcfState_Cluster *>;
vector<AP*> *m_APContainer = NetworkManager::Init()->GetAPContainer();    
if (m_APContainer->size() > 0)
{
	vector<AP *>::iterator it;
	for (it = m_APContainer->begin(); it != m_APContainer->end(); it++)
		Dcf_Cluster_Container->push_back(new DcfState_Cluster(*it));                  
}
Simulator::Init()->Schedule("wifi", 0.0, &DcfManager::StartDCF, this);      //����ʼ��Ϊһ���¼�
}


DcfManager::~DcfManager()
{
	vector<DcfState_Cluster *>::iterator iter;
	for (iter = Dcf_Cluster_Container->begin();
		iter != Dcf_Cluster_Container->end(); iter++)
	{
		delete *iter;
	}
	delete Dcf_Cluster_Container;
}

DcfState_Cluster * 
DcfManager::GetCluster(AP * m_ap)
{
	DcfState_Cluster * m_DC;
	vector<DcfState_Cluster *>::iterator it;	
	for (it = Dcf_Cluster_Container->begin(); it != Dcf_Cluster_Container->end(); it++)
	{
		if ((*it)->local_ap == m_ap)
		{	
			m_DC = *it;
		    break;
		}		
	}

	return m_DC;
}

void
DcfManager::StartDCF()
{     
	cout << "WIFI�������ڿ�ʼ!" << endl;
	vector<DcfState_Cluster *>::iterator iter1;
	DcfState_Cluster * oneDcfState_Cluster;
	for (iter1 = Dcf_Cluster_Container->begin(); iter1 != Dcf_Cluster_Container->end(); iter1++)
	{
	 oneDcfState_Cluster = *iter1;

	 oneDcfState_Cluster->StartCop();   //ÿ���ؿ�ʼ

//	 Simulator::Init()->Schedule("wifi", 0.0, &DcfState_Cluster::StartCop, oneDcfState_Cluster);  
	}

}
