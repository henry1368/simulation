


#ifndef  WIFI_BANDWIDTH_MANAGER_H_
#define  WIFI_BANDWIDTH_MANAGER_H_

#include <vector>

/*
Ŀǰ����802.11ac 5GƵ���й�����36 40 44 48 52 56 60 64 149 153 157 161 165�ܹ�13���ŵ��ܹ�260MHz�������ǲ�����
802.11ac֧�ִ�����20M��40M��80M��160M��������֧��80MHz�����д������ʱ��ÿ��wifiȺ�������һ�£�ÿ��WifiȺ�ڵ�5��
wifi���������ڸ��á�
ap0:�ŵ�36-48
ap1:�ŵ�44-56
ap2:�ŵ�52-64
ap3:�ŵ�149-161
ap4:�ŵ�153-165 
*/
//Ϊ�˷������ͬƵ���ţ������ŵ��ı�־λ��ap1�ı�־Ϊ0���ʾռ���ŵ�36-48���Դ�����

#define WIFI_BANDWIDTH    80                  //�趨wifi�ŵ�����Ϊ80MHz
#define RBs_FOR_WIFI  256         //wifi���ز�����80/0.3125(�ܴ���/���ز����)
#define WIFI_DL_LOW_FREQUENCY_0   5170	//MHz   ��36�����ŵ�����Ƶ��Ϊ5180M
#define WIFI_DL_LOW_FREQUENCY_1   5210 	//MHz   ��44�����ŵ�����Ƶ��Ϊ5220M
#define WIFI_DL_LOW_FREQUENCY_2   5250 	//MHz   ��52�����ŵ�����Ƶ��Ϊ5260M
#define WIFI_DL_LOW_FREQUENCY_3   5735 	//MHz   ��149�����ŵ�����Ƶ��Ϊ5745M
#define WIFI_DL_LOW_FREQUENCY_4   5755 	//MHz   ��153�����ŵ�����Ƶ��Ϊ5765M

class wifi_BandwidthManager {
public:
	wifi_BandwidthManager();
	wifi_BandwidthManager(double dlBw, int channelflag);
	virtual ~wifi_BandwidthManager();

	void SetDLSubChannels(std::vector<double> s); //�������ŵ�
	std::vector<double> GetDLSubChannels ();

	void SetDlBandwidth (double b);       //���д���
    double GetDlBandwidth ();
    
	int Getchannelflag();      //�ŵ�ѡ���־
	void Setchannelflag(int o);


	wifi_BandwidthManager* Copy();

	void Print ();

private:
	std::vector<double> m_dlSubChannels;       //�������ŵ�
	double m_dlBandwidth;                      //���д���
	int m_channelflag;                         //�ŵ�ѡ���־
      
};

#endif 
