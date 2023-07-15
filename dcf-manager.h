#ifndef DCF_MANAGER_H_
#define DCF_MANAGER_H_

/*
参考ns-3,该类实现的功能是控制整体（全部簇）进行发包
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
	DcfState_Cluster * GetCluster(AP * m_ap);               //取出某个AP对应的簇
//	vector<DcfState_Cluster *>* GetClusterContainer();      //返回所有的簇

	void StartDCF();                                      //开始所有的簇的仿真

private:
	DcfManager();
	vector<DcfState_Cluster *>* Dcf_Cluster_Container;      //该结构体包含了所有的簇
	static DcfManager *ptr;				                         //指针
};











#endif


