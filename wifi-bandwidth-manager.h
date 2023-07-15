


#ifndef  WIFI_BANDWIDTH_MANAGER_H_
#define  WIFI_BANDWIDTH_MANAGER_H_

#include <vector>

/*
目前关于802.11ac 5G频段中国开放36 40 44 48 52 56 60 64 149 153 157 161 165总共13个信道总共260MHz带宽，但是不连续
802.11ac支持带宽有20M、40M、80M、160M，本程序支持80MHz。进行带宽分配时，每个wifi群带宽分配一致，每个Wifi群内的5个
wifi带宽分配存在复用。
ap0:信道36-48
ap1:信道44-56
ap2:信道52-64
ap3:信道149-161
ap4:信道153-165 
*/
//为了方便计算同频干扰，设置信道的标志位如ap1的标志为0则表示占用信道36-48，以此类推

#define WIFI_BANDWIDTH    80                  //设定wifi信道带宽为80MHz
#define RBs_FOR_WIFI  256         //wifi子载波个数80/0.3125(总带宽/子载波间隔)
#define WIFI_DL_LOW_FREQUENCY_0   5170	//MHz   第36个子信道中心频率为5180M
#define WIFI_DL_LOW_FREQUENCY_1   5210 	//MHz   第44个子信道中心频率为5220M
#define WIFI_DL_LOW_FREQUENCY_2   5250 	//MHz   第52个子信道中心频率为5260M
#define WIFI_DL_LOW_FREQUENCY_3   5735 	//MHz   第149个子信道中心频率为5745M
#define WIFI_DL_LOW_FREQUENCY_4   5755 	//MHz   第153个子信道中心频率为5765M

class wifi_BandwidthManager {
public:
	wifi_BandwidthManager();
	wifi_BandwidthManager(double dlBw, int channelflag);
	virtual ~wifi_BandwidthManager();

	void SetDLSubChannels(std::vector<double> s); //下行子信道
	std::vector<double> GetDLSubChannels ();

	void SetDlBandwidth (double b);       //下行带宽
    double GetDlBandwidth ();
    
	int Getchannelflag();      //信道选择标志
	void Setchannelflag(int o);


	wifi_BandwidthManager* Copy();

	void Print ();

private:
	std::vector<double> m_dlSubChannels;       //下行子信道
	double m_dlBandwidth;                      //下行带宽
	int m_channelflag;                         //信道选择标志
      
};

#endif 
