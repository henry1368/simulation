
#ifndef  LTE_A_CHANNELPORT_H_
#define  LTE_A_CHANNELPORT_H_                      //����û����վ����AP֮��ϵ������

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
	LteChannel();         //����LTE�ŵ�    
	~LteChannel();

	UserEquipment* GetUe();
	void SetUe(UserEquipment* a);

	LtePropagationLossModel * GetLtePropagationLossModel();
	void SetLtePropagationLossModel(LtePropagationLossModel*b);

	//1===���վ����·������ʱ�洢�ı���������С�߶�ʱҪ�õ�
	double        thetaBS[nb_totalEnb];
	double        thetaMS[nb_totalEnb];
	double        elevation[nb_totalEnb];
	double        eNodeB_path_loss[nb_totalEnb];               //�û�����������֮���·����� 
	double        eNodeB_penetration_loss[nb_totalEnb];        //��͸���
	double        eNodeB_shadow_loss[nb_totalEnb];             //�洢��Ӱ˥��
	double        eNodeB_antennaGain[nb_totalEnb];             //��������
	bool          eNodeB_isLOS[nb_totalEnb];                   //�û������еĴ��վ�Ƿ��Ӿ��ж�
	double        eNodeB_LSPs[nb_totalEnb][5];                //�û�����������֮���LSPs

	//2===С��վ����·������ʱ�洢�ı���������С�߶�ʱҪ�õ�
	double         Helevation[nb_totalHenb];                   //������
	double         HethetaBS[nb_totalHenb];                    //�����
	double         HethetaMS[nb_totalHenb];
	double         HeNodeB_path_loss[nb_totalHenb];            //·����� 
	double         HeNodeB_penetration_loss[nb_totalHenb];     //��͸���
	double         HeNodeB_shadow_loss[nb_totalHenb];          //��Ӱ���
	double         HeNodeB_antennaGain[nb_totalHenb];          //��������
	bool           HeNodeB_isLOS[nb_totalHenb];                //�û�������С��վ�Ƿ��Ӿ��ж�
	double         HeNodeB_LSPs[nb_totalHenb][5];             //�û�����������֮���LSPs����


	//3===D2D��·�������               //����D2D�ն�
	double *D2D_Helevation;      
	double *D2D_thetaBS;
	double *D2D_thetaMS;
	double *D2D_elevation;
	double *D2D_eNodeB_path_loss;               //�û�����������֮���·����� 
	double *D2D_eNodeB_penetration_loss;        //��͸���
	double *D2D_eNodeB_shadow_loss;             //�洢��Ӱ˥��
	double *D2D_eNodeB_antennaGain;             //��������
	bool   *D2D_eNodeB_isLOS;                   //�û������еĴ��վ�Ƿ��Ӿ��ж�
	double **D2D_eNodeB_LSPs;                //�û�����������֮���LSPs



	//3��С��վ�Ĵ�߶ȡ�С�߶Ƚ��
	double    eNodeB_power_loss[nb_totalEnb + 1];                   //���վ�û�����������֮��Ĺ������ 
	double    HeNodeB_power_loss[nb_totalHenb];                      //С�û�������С��վ֮��Ĺ������ 

	MatrixXcd channel_Ecoefficients[nb_totalEnb][RBs_FOR_LTE];  // ������û��������������ڲ�ͬ��Ƶ���ϵ�Ƶ���ŵ�ϵ������(Nr��Nt)
	MatrixXcd channel_Hecoefficients[nb_totalHenb][RBs_FOR_LTE];          // ������û�������С��վ�ڲ�ͬ��Ƶ���ϵ�Ƶ���ŵ�ϵ������(Nr��Nt)

      //Matrix<complex<double>, Nr, Nt>[nb_totalHenb][RBs_FOR_WIFI]�����ڴ治����


private:

	LtePropagationLossModel * m_LtePropagationLossModel;
	UserEquipment* m_ue;             //�����û�
};
#endif

