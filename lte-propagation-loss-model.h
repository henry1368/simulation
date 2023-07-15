

#ifndef LTE_PROPAGATIONLOSSMODEL_H_
#define LTE_PROPAGATIONLOSSMODEL_H_

#include "parameter.h"
#include<vector>

class NetworkNode;
class ENodeB;
class HeNodeB;
class UserEquipment;
using namespace std;

class LtePropagationLossModel {
public:
	LtePropagationLossModel();
	virtual ~LtePropagationLossModel();

	enum PropagationType                              //传播类型
	{
		TYPE_UMi,
		TYPE_UMa,
		TYPE_RMa,  
		TYPE_SMa,
		TYPE_TC1,
		TYPE_TC2,
	};

	void SetPropagationType(PropagationType t);
	PropagationType GetPropagationType();


	struct ChannMedi{
		int             num_paths_m;//每条链路的多径数
		int             num_rays_m[24];//每条多径的子径数	
		double    fc_m;
		double     pathLoss_m;       //路径损耗
		double     shadowFading_m;   //阴影衰落
		//double     travelAngle_m ; //用户移动方向
		double     arrival_angles_m[20][24];//到达角
		double     depart_angles_m[20][24]; //离开角
		double     initial_phase_vv_m[20][24];//天线垂直--垂直相位
		double     initial_phase_vh_m[20][24];
		double     initial_phase_hv_m[20][24];
		double     initial_phase_hh_m[20][24];
		double     initial_phase_vv_LOS_m;//天线垂直方向上视距相位
		double     initial_phase_hh_LOS_m;
		double     delay_clusters_m[20 + 4];//初始延时径
		double     power_path_ini_m[20 + 4];//初始功率
		double     XPR_m[20][24];            //存储交叉极化比（考虑极化情况下）
	};

	ChannMedi E_medi;
	ChannMedi HE_medi;

	vector<ChannMedi> eNodeB_medi;//存储一个用户到所有扇区计算小尺度所需的中间变量
	vector<ChannMedi>HeNodeB_medi;//存储一个用户到所有小基站计算小尺度所需的中间变量

	//===信道计算需要确定的一些参数
	int              fftSize;
	int              numsubcarrier;      //子载波数	  	
	double           fc;                 //中心频率
	double           Fre_interval;       //子载波间隔
	double           sample_time;        //采样间隔
	long             sample_rate;        //信道输出采样间隔	       ********
	double           time;            //计算信道的时刻
	int              Bs_index;           //基站编号
	int              Ms_index;           //用户编号
	double	         pathLoss;
	double	         shadowFading;
	double           travelAngle;         //小尺度计算时添加，保存UE移动角度
	double           travelSpeed;         //大尺度计算是根据场景确定参数，小尺度计算时用到，保存UE移动速率



	virtual  void  Compute_PowerLoss(UserEquipment* m_src, vector<ENodeB*> *m_dst) = 0;    //宏基站大尺度衰落
	virtual  void  Compute_PowerLoss(UserEquipment* m_src, vector<HeNodeB*> *m_dst) = 0;   //小基站大尺度衰落

	virtual bool  Compute_isLOS(UserEquipment* m_src, NetworkNode* m_dst) = 0;                //视距计算
	virtual double Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst) = 0;             //路径损耗
	virtual double Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst) = 0;      //穿透损耗
	virtual double Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst) = 0;         //阴影衰落
	virtual double Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst) = 0;          //天线增益


	//5 == =计算小尺度的相关中间变量
	//径时延
	double      *delay_path;
	//径功率
	double      *power_path;
	//double      *power_path_los;
	double       max_power_los;//直视径下的最大功率
	//发射角和到达角
	double       c_factor;
	double       **offset_angle;
	double       *arrival_angles_ini;
	double       *depart_angles_ini;
	//对路径进行分簇处理 
	int          LN_size;//最强两簇分簇后，射线簇总数为 LN_size=clusterNum+4		
	//用于功率、到达角、发射角的统一高斯随机变量
	double       gaussRand[20];
	double       delay_spread, AoA_spread, AoD_spread, K_factor, K_factor_db;   //计算大尺度衰落的LSPs参数
	int          clusterNum, LOSflag, clusrerNum_ini;                           //多径数目 1:存在直视径, 0:不存在直视径
	double       XPR_mu, XPR_sigma, Caod, Caoa, zita;
	double	     rtau;//时延缩放参数
	double       phi_AoA_LOS;
	double       phi_AoD_LOS;
	double       ASD_DS, ASA_DS, ASA_SF, ASD_SF, DS_SF, ASD_ASA, ASD_K, ASA_K, DS_K, SF_K;
	double       DS_dis, ASD_dis, ASA_dis, SF_dis, K_dis;
	double       max_power;//最大功率
	int          max_idx, max_idx_2nd;     //强功率簇索引

	//6 == =计算小尺度所调用函数，基本按照协议流程的循序
	void       setEPoint(UserEquipment* Ms, ENodeB * Bs);//大基站  获取当前用户指示扇区及一些参数
	void       setHePoint(UserEquipment* Ms, HeNodeB * Bs);//小基站  获取当前用户指示小基站序号及一些参数

	void         initial();//初始化信道相关参量
	void         set_small_scale_parameter(char * str);
	void        initial_Array();//对后面计算需要的数组进行初始化。最后记得函数释放内存空间

	void        generate_delay();//计算cluster的时延
	void       generate_Gauss();//生成高斯随机变量，用于功率、到达角、发射角时所需
	void      generate_powers();//产生信道中的功率
	void      generate_c();
	void      generate_AoAs(); //产生到达角度
	void      generate_AoDs();
	void     set_SubClusters();//对1，2的强功率簇进行分簇处理，AOA与AOD随机配对
	void     generate_initial_phase();//大基站极化初始相位
	void     generate_XPR();

	 void  calculateNodeB_medi(UserEquipment* m_src, vector<ENodeB*> * m_dst);                     //小尺度中间变量
	 void  calculateHeNodeB_medi(UserEquipment* m_src, vector<HeNodeB*> * m_dst);                  
	virtual void Compute_SmallscaleFading(UserEquipment* m_src, vector<ENodeB*> * m_dst, double m_time);       //小尺度衰落
	virtual void Compute_SmallscaleFading(UserEquipment* m_src, vector<HeNodeB*> * m_dst, double m_time);       //小尺度衰落

private:
	PropagationType m_channelType;
};

#endif 




