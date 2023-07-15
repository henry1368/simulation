#pragma once
#ifndef  CHANNELPORT_H_
#define CHANNELPORT_H_                      //算出用户与基站或者AP之间系数矩阵
#include <vector>
#include <complex>
#include "parameter.h"
#include<cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class UserEquipment;

	struct ChannMedi{
		int             num_paths_m;//每条链路的多径数
		int             num_rays_m[24] ;//每条多径的子径数	
		double    fc_m;
		double     pathLoss_m;//路径损耗
		double     shadowFading_m;//阴影衰落
		//double     travelAngle_m ;//用户移动方向
		double     arrival_angles_m[20][24] ;//到达角
		double     depart_angles_m[20][24] ;//离开角
		double     initial_phase_vv_m[20][24] ;//天线垂直--垂直相位
		double     initial_phase_vh_m[20][24];
		double     initial_phase_hv_m[20][24] ;
		double     initial_phase_hh_m[20][24] ;
		double     initial_phase_vv_LOS_m;//天线垂直方向上视距相位
		double     initial_phase_hh_LOS_m;
		double     delay_clusters_m[20 + 4] ;//初始延时径
		double     power_path_ini_m[20 + 4] ;//初始功率
		double     XPR_m[20][24] ;            //存储交叉极化比（考虑极化情况下）
	
	};


class ChannelPort
{

public:
	ChannelPort();
	ChannelPort( char * type);          //生成LTE或者WIFI的信道     
	~ChannelPort();
	
	
	UserEquipment* ue;      //所属用户
	ChannMedi E_medi;
	ChannMedi HE_medi;
	ChannMedi AP_medi;
	vector<ChannMedi> eNodeB_medi;//存储一个用户到所有扇区计算小尺度所需的中间变量
	vector<ChannMedi>HeNodeB_medi;//存储一个用户到所有小基站计算小尺度所需的中间变量
	vector<ChannMedi> Wifi_medi;//存储一个用户到所有AP群计算小尺度所需的中间变量

	void  calculateNodeB_medi();//产生当前用户到所有扇区的信道系数的中间变量并储存
	void  calculateHeNodeB_medi();//产生当前用户到所有小基站的信道系数的中间变量并储存
	void  calculateAP_medi();//产生当前用户到所有AP群的信道系数的中间变量并储存
	//1 ===信道在初始化时需要确定的一些参数及与FFT变换相关的参数
	//(1.城市微小区 UMi 2.城市宏小区 UMa 3.农村宏小区 RMa 4.郊区宏小区 SMa )
	
	int              fftSize;     	
	int              numsubcarrier;      //子载波数	  	
	double           fc;//中心频率
	double           wififc;
	double           Fre_interval;        //子载波间隔
	double           sample_time;//采样间隔
	long             sample_rate;        //信道输出采样间隔	       ********
	int              TTItime;               //计算信道的时刻
	int              Sectorid;//扇区号
	double	         pathLoss;
	double	         shadowFading;
	double           travelAngle;         //小尺度计算时添加，保存UE移动角度
	double           travelSpeed;         //大尺度计算是根据场景确定参数，小尺度计算时用到，保存UE移动速率
	
	
	//2===大基站计算路径增益时存储的变量，计算小尺度时要用到
	double        thetaBS[nb_cell];
	double        thetaMS[nb_cell];
	double        elevation[nb_cell];
	double        eNodeB_path_loss[nb_cell];               //用户到所有扇区之间的路径损耗 
	double        eNodeB_shadow_loss[nb_cell];            //存储阴影衰落，中间变量数组
	 bool         eNodeB_isLOS[nb_cell];               //用户到所有的大基站是否视距判定
	double        eNodeB_LSPs[nb_cell][5];            //用户到所有扇区之间的LSPs

	double         eNodeB_Ksi[nb_cell][5];          //空间滤波Ksi


	//3===小基站计算路径增益时存储的变量，计算小尺度时要用到
	double         Helevation[nb_totalHenb];//小基站的俯仰角
	double         HethetaBS[nb_totalHenb];
	double         HethetaMS[nb_totalHenb];
	double         HeNodeB_path_loss[nb_totalHenb];            //用户到所有小基站之间的路径损耗 	
	double         HeNodeB_shadow_loss[nb_totalHenb];
	bool           HeNodeB_isLOS[nb_totalHenb];                //用户到所有小基站是否视距判定
	double         HeNodeB_LSPs[nb_totalHenb][5];             //用户到所有扇区之间的LSPs参数

	double         HeNodeB_Ksi[nb_totalHenb][5];          //空间滤波Ksi

	//4===大小基站计算大尺度的调用函数
	
	void Compute_Ksi_HeNodeBpathloss();
	void Compute_Ksi_eNodeBpathloss();                                    //只根据Ksi来更新大尺度

	void generateLSPs(char* str, bool LOSflag, double *lsp,double* ski);  //根据LOSFLAG计算LSPs存储到lsp中

	void Compute_HeNodeBpathloss();                               //计算小基站的大尺度衰落
	void Compute_eNodeBpathloss();                                //计算大基站的大尺度衰落



	//==================以下计算小尺度的相关中间变量和函数===========/
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
	double       delay_spread, AoA_spread, AoD_spread, K_factor, K_factor_db;//计算大尺度衰落的LSPs参数
	int             clusterNum, LOSflag, clusrerNum_ini;//多径数目 1:存在直视径, 0:不存在直视径
	double       XPR_mu, XPR_sigma, Caod, Caoa, zita;
	double	     rtau;//时延缩放参数
	double       phi_AoA_LOS;
	double       phi_AoD_LOS;
	double       ASD_DS, ASA_DS, ASA_SF, ASD_SF, DS_SF, ASD_ASA, ASD_K, ASA_K, DS_K, SF_K;
	double       DS_dis, ASD_dis, ASA_dis, SF_dis, K_dis;
	double       max_power;//最大功率
	int          max_idx, max_idx_2nd;     //强功率簇索引

	
	//6 == =计算小尺度所调用函数，基本按照协议流程的循序
	void       setEPoint(char sid);//大基站  获取当前用户指示扇区及一些参数
	void       setHePoint(char sid);//小基站  获取当前用户指示小基站序号及一些参数

	void         initial();//初始化信道相关参量
	void         set_small_scale_parameter(char * ch);
	void        initial_Array();//对后面计算需要的数组进行初始化。最后记得函数释放内存空间

	void        generate_ELSPsigma();// 大基站 得到到某一固定扇区的时延扩展（一个小区的三个扇区参数相同），角度扩展等。
	void        generate_HeLSPsigma();// 小基站 得到到某一固定小基站的时延扩展，角度扩展等。

	void        generate_delay();//计算cluster的时延
	void       generate_Gauss();//生成高斯随机变量，用于功率、到达角、发射角时所需
	void      generate_powers();//产生信道中的功率
	void      generate_c();
	void      generate_AoAs(); //产生到达角度
	void      generate_AoDs();
	void     set_SubClusters();//对1，2的强功率簇进行分簇处理，AOA与AOD随机配对
	void     generate_initial_phase();//大基站极化初始相位
	void     generate_XPR();

	//7 == =大小基站调用小尺度计算函数
	//大基站计算小尺度
	void calculateNodeBSmallFading(int mscene, int time);      //产生当前用户到所有扇区的信道系数
	//小基站计算小尺度
	void  calculateHeNodeBSmallFading( int time); // 产生当前用户到所有扇区的信道系数
//=====================以上为计算小尺度================
//===大小基站的大、小尺度结果
//大小基站的大尺度结果
double         eNodeB_power_loss[nb_totalEnb+1];               //大基站用户到所有扇区之间的功率损耗 
double         HeNodeB_power_loss[nb_totalHenb];    //小用户到所有小基站之间的功率损耗 
//大小基站的小尺度结果
complex<double>  channel_coefficients_time[Nr][Nt][24];   //时域信道响应系数【基站天线】【用户天线】【射线簇数（最强簇分簇后）】


MatrixXcd channel_Ecoefficients[nb_cell * nb_sector][RBs_FOR_LTE]; // 储存该用户到所有扇区的在不同载频点上的频域信道系数矩阵(Nr×Nt)

MatrixXcd channel_Hecoefficients[nb_totalHenb][RBs_FOR_LTE]; // 储存该用户到所有小基站在不同载频点上的频域信道系数矩阵(Nr×Nt)




///////////////////////Wifi大小尺度的所有参数及函数//////////////////////
bool      AP_isLOS[nb_totalAP];
double  AP_path_loss[nb_totalAP];
double  AP_shadow_loss[nb_totalAP];
double         APelevation[nb_totalAP];//小基站的俯仰角
double         APthetaBS[nb_totalAP];
double         APthetaMS[nb_totalAP];
double         AP_LSPs[nb_totalAP][5];          //用户到所有扇区之间的LSPs参数

double AP_power_loss[nb_totalAP];
void      Compute_Wifipathloss();             //计算Wifi的Ap群的路损

void setAPPoint(char sid);
void generate_APLSPsigma();

void  calculateAPSmallFading(int time);//产生当前用户到所有AP群的信道系数


MatrixXcd channel_APcoefficients[nb_totalAP][RBs_FOR_WIFI];    //储存该用户到所有AP的在不同载频点上的信道矩阵(Nr×Nt)

                                                       //Matrix<complex<double>, Nr, Nt>[nb_totalAP][RBs_FOR_WIFI]报错，内存不对齐


};
#endif

