#pragma once
#ifndef  CHANNELPORT_H_
#define CHANNELPORT_H_                      //����û����վ����AP֮��ϵ������
#include <vector>
#include <complex>
#include "parameter.h"
#include<cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class UserEquipment;

	struct ChannMedi{
		int             num_paths_m;//ÿ����·�Ķྶ��
		int             num_rays_m[24] ;//ÿ���ྶ���Ӿ���	
		double    fc_m;
		double     pathLoss_m;//·�����
		double     shadowFading_m;//��Ӱ˥��
		//double     travelAngle_m ;//�û��ƶ�����
		double     arrival_angles_m[20][24] ;//�����
		double     depart_angles_m[20][24] ;//�뿪��
		double     initial_phase_vv_m[20][24] ;//���ߴ�ֱ--��ֱ��λ
		double     initial_phase_vh_m[20][24];
		double     initial_phase_hv_m[20][24] ;
		double     initial_phase_hh_m[20][24] ;
		double     initial_phase_vv_LOS_m;//���ߴ�ֱ�������Ӿ���λ
		double     initial_phase_hh_LOS_m;
		double     delay_clusters_m[20 + 4] ;//��ʼ��ʱ��
		double     power_path_ini_m[20 + 4] ;//��ʼ����
		double     XPR_m[20][24] ;            //�洢���漫���ȣ����Ǽ�������£�
	
	};


class ChannelPort
{

public:
	ChannelPort();
	ChannelPort( char * type);          //����LTE����WIFI���ŵ�     
	~ChannelPort();
	
	
	UserEquipment* ue;      //�����û�
	ChannMedi E_medi;
	ChannMedi HE_medi;
	ChannMedi AP_medi;
	vector<ChannMedi> eNodeB_medi;//�洢һ���û���������������С�߶�������м����
	vector<ChannMedi>HeNodeB_medi;//�洢һ���û�������С��վ����С�߶�������м����
	vector<ChannMedi> Wifi_medi;//�洢һ���û�������APȺ����С�߶�������м����

	void  calculateNodeB_medi();//������ǰ�û��������������ŵ�ϵ�����м����������
	void  calculateHeNodeB_medi();//������ǰ�û�������С��վ���ŵ�ϵ�����м����������
	void  calculateAP_medi();//������ǰ�û�������APȺ���ŵ�ϵ�����м����������
	//1 ===�ŵ��ڳ�ʼ��ʱ��Ҫȷ����һЩ��������FFT�任��صĲ���
	//(1.����΢С�� UMi 2.���к�С�� UMa 3.ũ���С�� RMa 4.������С�� SMa )
	
	int              fftSize;     	
	int              numsubcarrier;      //���ز���	  	
	double           fc;//����Ƶ��
	double           wififc;
	double           Fre_interval;        //���ز����
	double           sample_time;//�������
	long             sample_rate;        //�ŵ�����������	       ********
	int              TTItime;               //�����ŵ���ʱ��
	int              Sectorid;//������
	double	         pathLoss;
	double	         shadowFading;
	double           travelAngle;         //С�߶ȼ���ʱ��ӣ�����UE�ƶ��Ƕ�
	double           travelSpeed;         //��߶ȼ����Ǹ��ݳ���ȷ��������С�߶ȼ���ʱ�õ�������UE�ƶ�����
	
	
	//2===���վ����·������ʱ�洢�ı���������С�߶�ʱҪ�õ�
	double        thetaBS[nb_cell];
	double        thetaMS[nb_cell];
	double        elevation[nb_cell];
	double        eNodeB_path_loss[nb_cell];               //�û�����������֮���·����� 
	double        eNodeB_shadow_loss[nb_cell];            //�洢��Ӱ˥�䣬�м��������
	 bool         eNodeB_isLOS[nb_cell];               //�û������еĴ��վ�Ƿ��Ӿ��ж�
	double        eNodeB_LSPs[nb_cell][5];            //�û�����������֮���LSPs

	double         eNodeB_Ksi[nb_cell][5];          //�ռ��˲�Ksi


	//3===С��վ����·������ʱ�洢�ı���������С�߶�ʱҪ�õ�
	double         Helevation[nb_totalHenb];//С��վ�ĸ�����
	double         HethetaBS[nb_totalHenb];
	double         HethetaMS[nb_totalHenb];
	double         HeNodeB_path_loss[nb_totalHenb];            //�û�������С��վ֮���·����� 	
	double         HeNodeB_shadow_loss[nb_totalHenb];
	bool           HeNodeB_isLOS[nb_totalHenb];                //�û�������С��վ�Ƿ��Ӿ��ж�
	double         HeNodeB_LSPs[nb_totalHenb][5];             //�û�����������֮���LSPs����

	double         HeNodeB_Ksi[nb_totalHenb][5];          //�ռ��˲�Ksi

	//4===��С��վ�����߶ȵĵ��ú���
	
	void Compute_Ksi_HeNodeBpathloss();
	void Compute_Ksi_eNodeBpathloss();                                    //ֻ����Ksi�����´�߶�

	void generateLSPs(char* str, bool LOSflag, double *lsp,double* ski);  //����LOSFLAG����LSPs�洢��lsp��

	void Compute_HeNodeBpathloss();                               //����С��վ�Ĵ�߶�˥��
	void Compute_eNodeBpathloss();                                //������վ�Ĵ�߶�˥��



	//==================���¼���С�߶ȵ�����м�����ͺ���===========/
	//5 == =����С�߶ȵ�����м����
	//��ʱ��
	double      *delay_path;
	//������
	double      *power_path;
	//double      *power_path_los;
	double       max_power_los;//ֱ�Ӿ��µ������
	//����Ǻ͵����
	double       c_factor;
	double       **offset_angle;
	double       *arrival_angles_ini;
	double       *depart_angles_ini;
	//��·�����зִش��� 
	int          LN_size;//��ǿ���طִغ����ߴ�����Ϊ LN_size=clusterNum+4		
	//���ڹ��ʡ�����ǡ�����ǵ�ͳһ��˹�������
	double       gaussRand[20];
	double       delay_spread, AoA_spread, AoD_spread, K_factor, K_factor_db;//�����߶�˥���LSPs����
	int             clusterNum, LOSflag, clusrerNum_ini;//�ྶ��Ŀ 1:����ֱ�Ӿ�, 0:������ֱ�Ӿ�
	double       XPR_mu, XPR_sigma, Caod, Caoa, zita;
	double	     rtau;//ʱ�����Ų���
	double       phi_AoA_LOS;
	double       phi_AoD_LOS;
	double       ASD_DS, ASA_DS, ASA_SF, ASD_SF, DS_SF, ASD_ASA, ASD_K, ASA_K, DS_K, SF_K;
	double       DS_dis, ASD_dis, ASA_dis, SF_dis, K_dis;
	double       max_power;//�����
	int          max_idx, max_idx_2nd;     //ǿ���ʴ�����

	
	//6 == =����С�߶������ú�������������Э�����̵�ѭ��
	void       setEPoint(char sid);//���վ  ��ȡ��ǰ�û�ָʾ������һЩ����
	void       setHePoint(char sid);//С��վ  ��ȡ��ǰ�û�ָʾС��վ��ż�һЩ����

	void         initial();//��ʼ���ŵ���ز���
	void         set_small_scale_parameter(char * ch);
	void        initial_Array();//�Ժ��������Ҫ��������г�ʼ�������ǵú����ͷ��ڴ�ռ�

	void        generate_ELSPsigma();// ���վ �õ���ĳһ�̶�������ʱ����չ��һ��С������������������ͬ�����Ƕ���չ�ȡ�
	void        generate_HeLSPsigma();// С��վ �õ���ĳһ�̶�С��վ��ʱ����չ���Ƕ���չ�ȡ�

	void        generate_delay();//����cluster��ʱ��
	void       generate_Gauss();//���ɸ�˹������������ڹ��ʡ�����ǡ������ʱ����
	void      generate_powers();//�����ŵ��еĹ���
	void      generate_c();
	void      generate_AoAs(); //��������Ƕ�
	void      generate_AoDs();
	void     set_SubClusters();//��1��2��ǿ���ʴؽ��зִش���AOA��AOD������
	void     generate_initial_phase();//���վ������ʼ��λ
	void     generate_XPR();

	//7 == =��С��վ����С�߶ȼ��㺯��
	//���վ����С�߶�
	void calculateNodeBSmallFading(int mscene, int time);      //������ǰ�û��������������ŵ�ϵ��
	//С��վ����С�߶�
	void  calculateHeNodeBSmallFading( int time); // ������ǰ�û��������������ŵ�ϵ��
//=====================����Ϊ����С�߶�================
//===��С��վ�Ĵ�С�߶Ƚ��
//��С��վ�Ĵ�߶Ƚ��
double         eNodeB_power_loss[nb_totalEnb+1];               //���վ�û�����������֮��Ĺ������ 
double         HeNodeB_power_loss[nb_totalHenb];    //С�û�������С��վ֮��Ĺ������ 
//��С��վ��С�߶Ƚ��
complex<double>  channel_coefficients_time[Nr][Nt][24];   //ʱ���ŵ���Ӧϵ������վ���ߡ����û����ߡ������ߴ�������ǿ�طִغ󣩡�


MatrixXcd channel_Ecoefficients[nb_cell * nb_sector][RBs_FOR_LTE]; // ������û��������������ڲ�ͬ��Ƶ���ϵ�Ƶ���ŵ�ϵ������(Nr��Nt)

MatrixXcd channel_Hecoefficients[nb_totalHenb][RBs_FOR_LTE]; // ������û�������С��վ�ڲ�ͬ��Ƶ���ϵ�Ƶ���ŵ�ϵ������(Nr��Nt)




///////////////////////Wifi��С�߶ȵ����в���������//////////////////////
bool      AP_isLOS[nb_totalAP];
double  AP_path_loss[nb_totalAP];
double  AP_shadow_loss[nb_totalAP];
double         APelevation[nb_totalAP];//С��վ�ĸ�����
double         APthetaBS[nb_totalAP];
double         APthetaMS[nb_totalAP];
double         AP_LSPs[nb_totalAP][5];          //�û�����������֮���LSPs����

double AP_power_loss[nb_totalAP];
void      Compute_Wifipathloss();             //����Wifi��ApȺ��·��

void setAPPoint(char sid);
void generate_APLSPsigma();

void  calculateAPSmallFading(int time);//������ǰ�û�������APȺ���ŵ�ϵ��


MatrixXcd channel_APcoefficients[nb_totalAP][RBs_FOR_WIFI];    //������û�������AP���ڲ�ͬ��Ƶ���ϵ��ŵ�����(Nr��Nt)

                                                       //Matrix<complex<double>, Nr, Nt>[nb_totalAP][RBs_FOR_WIFI]�����ڴ治����


};
#endif

