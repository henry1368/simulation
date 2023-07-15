
#ifndef  LTE_A_CHANNELPORT_H_
#define  LTE_A_CHANNELPORT_H_                      //算出用户与基站或者AP之间系数矩阵

#include "string.h"
#include "parameter.h"
#include "lte-bandwidth-manager.h"
#include "lte-propagation-loss-model.h"
#include "urban-macro-propagation-model.h"
#include "urban-micro-propagation-model.h"
#include "MadridGrid-propagation-model.h"
#include <vector>
#include <complex>
#include "parameter.h"
#include<cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class UserEquipment;
class LtePropagationLossModel;


class LteChannel
{

public:
	LteChannel();         //生成LTE信道    
	~LteChannel();

	UserEquipment* GetUe();
	void SetUe(UserEquipment* a);

	LtePropagationLossModel * GetLtePropagationLossModel();
	void SetLtePropagationLossModel(LtePropagationLossModel*b);

	//1===大基站计算路径增益时存储的变量，计算小尺度时要用到
	double        thetaBS[nb_totalEnb];
	double        thetaMS[nb_totalEnb];
	double        elevation[nb_totalEnb];
	double        eNodeB_path_loss[nb_totalEnb];               //用户到所有扇区之间的路径损耗 
	double        eNodeB_penetration_loss[nb_totalEnb];        //穿透损耗
	double        eNodeB_shadow_loss[nb_totalEnb];             //存储阴影衰落
	double        eNodeB_antennaGain[nb_totalEnb];             //天线增益
	bool          eNodeB_isLOS[nb_totalEnb];                   //用户到所有的大基站是否视距判定
	double        eNodeB_LSPs[nb_totalEnb][5];                //用户到所有扇区之间的LSPs

	//2===小基站计算路径增益时存储的变量，计算小尺度时要用到
	double         Helevation[nb_totalHenb];                   //俯仰角
	double         HethetaBS[nb_totalHenb];                    //到达角
	double         HethetaMS[nb_totalHenb];
	double         HeNodeB_path_loss[nb_totalHenb];            //路径损耗 
	double         HeNodeB_penetration_loss[nb_totalHenb];     //穿透损耗
	double         HeNodeB_shadow_loss[nb_totalHenb];          //阴影损耗
	double         HeNodeB_antennaGain[nb_totalHenb];          //天线增益
	bool           HeNodeB_isLOS[nb_totalHenb];                //用户到所有小基站是否视距判定
	double         HeNodeB_LSPs[nb_totalHenb][5];             //用户到所有扇区之间的LSPs参数


	//3===D2D链路计算变量               //仅在D2D收端
	double *D2D_Helevation;      
	double *D2D_thetaBS;
	double *D2D_thetaMS;
	double *D2D_elevation;
	double *D2D_eNodeB_path_loss;               //用户到所有扇区之间的路径损耗 
	double *D2D_eNodeB_penetration_loss;        //穿透损耗
	double *D2D_eNodeB_shadow_loss;             //存储阴影衰落
	double *D2D_eNodeB_antennaGain;             //天线增益
	bool   *D2D_eNodeB_isLOS;                   //用户到所有的大基站是否视距判定
	double **D2D_eNodeB_LSPs;                //用户到所有扇区之间的LSPs



	//3大小基站的大尺度、小尺度结果
	double    eNodeB_power_loss[nb_totalEnb + 1];                   //大基站用户到所有扇区之间的功率损耗 
	double    HeNodeB_power_loss[nb_totalHenb];                      //小用户到所有小基站之间的功率损耗 

	MatrixXcd channel_Ecoefficients[nb_totalEnb][RBs_FOR_LTE];  // 储存该用户到所有扇区的在不同载频点上的频域信道系数矩阵(Nr×Nt)
	MatrixXcd channel_Hecoefficients[nb_totalHenb][RBs_FOR_LTE];          // 储存该用户到所有小基站在不同载频点上的频域信道系数矩阵(Nr×Nt)

      //Matrix<complex<double>, Nr, Nt>[nb_totalHenb][RBs_FOR_WIFI]报错，内存不对齐


private:

	LtePropagationLossModel * m_LtePropagationLossModel;
	UserEquipment* m_ue;             //所属用户
};
#endif

