

#ifndef WIFI_PROPAGATIONLOSSMODEL_H_
#define WIFI_PROPAGATIONLOSSMODEL_H_

#include <vector>

class NetworkNode;
class AP;
class UserEquipment;
using namespace std;

class WifiPropagationLossModel {
public:
	WifiPropagationLossModel();
	virtual ~WifiPropagationLossModel();

	enum PropagationType                              //��������
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
		int             num_paths_m;//ÿ����·�Ķྶ��
		int             num_rays_m[24];//ÿ���ྶ���Ӿ���	
		double    fc_m;
		double     pathLoss_m;       //·�����
		double     shadowFading_m;   //��Ӱ˥��
		//double     travelAngle_m ; //�û��ƶ�����
		double     arrival_angles_m[20][24];//�����
		double     depart_angles_m[20][24]; //�뿪��
		double     initial_phase_vv_m[20][24];//���ߴ�ֱ--��ֱ��λ
		double     initial_phase_vh_m[20][24];
		double     initial_phase_hv_m[20][24];
		double     initial_phase_hh_m[20][24];
		double     initial_phase_vv_LOS_m;//���ߴ�ֱ�������Ӿ���λ
		double     initial_phase_hh_LOS_m;
		double     delay_clusters_m[20 + 4];//��ʼ��ʱ��
		double     power_path_ini_m[20 + 4];//��ʼ����
		double     XPR_m[20][24];            //�洢���漫���ȣ����Ǽ�������£�
	};

	ChannMedi A_medi;

	vector<ChannMedi> AP_medi;//�洢һ���û�������AP����С�߶�������м����


	//===�ŵ�������Ҫȷ����һЩ����
	int              fftSize;
	int              numsubcarrier;      //���ز���	  	
	double           fc;                 //����Ƶ��
	double           Fre_interval;       //���ز����
	double           sample_time;        //�������
	long             sample_rate;        //�ŵ�����������	       ********
	double           time;            //�����ŵ���ʱ��
	int              Bs_index;           //��վ���
	int              Ms_index;           //�û����
	double	         pathLoss;
	double	         shadowFading;
	double           travelAngle;         //С�߶ȼ���ʱ���ӣ�����UE�ƶ��Ƕ�
	double           travelSpeed;         //��߶ȼ����Ǹ��ݳ���ȷ��������С�߶ȼ���ʱ�õ�������UE�ƶ�����



 void  Compute_PowerLoss(UserEquipment* m_src, vector<AP*> *m_dst) ;    //���վ��߶�˥��

 bool   Compute_isLOS(UserEquipment* m_src, AP* m_dst) ;                //�Ӿ����
 double Compute_PathLoss(UserEquipment* m_src, AP* m_dst) ;             //·�����
 double Compute_PenetrationLoss(UserEquipment* m_src, AP* m_dst) ;      //��͸���
 double Compute_ShadowFading(UserEquipment* m_src, AP* m_dst) ;         //��Ӱ˥��
 double Compute_AntennaGain(UserEquipment* m_src, AP* m_dst) ;          //��������


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
	double       delay_spread, AoA_spread, AoD_spread, K_factor, K_factor_db;   //�����߶�˥���LSPs����
	int          clusterNum, LOSflag, clusrerNum_ini;                           //�ྶ��Ŀ 1:����ֱ�Ӿ�, 0:������ֱ�Ӿ�
	double       XPR_mu, XPR_sigma, Caod, Caoa, zita;
	double	     rtau;//ʱ�����Ų���
	double       phi_AoA_LOS;
	double       phi_AoD_LOS;
	double       ASD_DS, ASA_DS, ASA_SF, ASD_SF, DS_SF, ASD_ASA, ASD_K, ASA_K, DS_K, SF_K;
	double       DS_dis, ASD_dis, ASA_dis, SF_dis, K_dis;
	double       max_power;//�����
	int          max_idx, max_idx_2nd;     //ǿ���ʴ�����

	//6 == =����С�߶������ú�������������Э�����̵�ѭ��
	void       setEPoint(UserEquipment* Ms, AP * Bs); //��ȡ��·��һЩ����
	void       initial();//��ʼ���ŵ���ز���
	void       set_small_scale_parameter();
	void       initial_Array();//�Ժ��������Ҫ��������г�ʼ�������ǵú����ͷ��ڴ�ռ�
	void      generate_delay();//����cluster��ʱ��
	void      generate_Gauss();//���ɸ�˹������������ڹ��ʡ�����ǡ������ʱ����
	void      generate_powers();//�����ŵ��еĹ���
	void      generate_c();
	void      generate_AoAs(); //��������Ƕ�
	void      generate_AoDs();
	void     set_SubClusters();//��1��2��ǿ���ʴؽ��зִش�����AOA��AOD������
	void     generate_initial_phase();//���վ������ʼ��λ
	void     generate_XPR();

	virtual void  calculateAP_medi(UserEquipment* m_src, vector<AP*> * m_dst);                     //С�߶��м����

	virtual void Compute_SmallscaleFading(UserEquipment* m_src, vector<AP*> * m_dst, double m_time);       //С�߶�˥��
	

private:
	PropagationType m_channelType;
};

#endif 



