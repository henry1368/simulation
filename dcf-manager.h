#ifndef DCF_MANAGER_H_
#define DCF_MANAGER_H_

/*
�ο�ns-3,����ʵ�ֵĹ����ǿ������壨ȫ���أ����з���
*/


#include  "vector"
using namespace std;
class DcfState_Cluster;
class AP;
class DcfManager
{
public:

	static  DcfManager*
		Init()
	{
			if (ptr == NULL)
			{
				ptr = new DcfManager;          
			}
			return ptr;
		}


	~DcfManager();
	DcfState_Cluster * GetCluster(AP * m_ap);               //ȡ��ĳ��AP��Ӧ�Ĵ�
//	vector<DcfState_Cluster *>* GetClusterContainer();      //�������еĴ�

	void StartDCF();                                      //��ʼ���еĴصķ���

private:
	DcfManager();
	vector<DcfState_Cluster *>* Dcf_Cluster_Container;      //�ýṹ����������еĴ�
	static DcfManager *ptr;				                         //ָ��
};











#endif


