

#include "wifi-bandwidth-manager.h"
#include <stdio.h>
#include <iostream>
#include "parameter.h"



wifi_BandwidthManager::wifi_BandwidthManager()
{
}

wifi_BandwidthManager::wifi_BandwidthManager( double dlBw,  int channelflag)
{

  m_dlBandwidth = dlBw;
  m_channelflag = channelflag % 5;
  m_dlSubChannels.clear();

  if (channelflag %5 == 0)
    {
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
        {
    	  m_dlSubChannels.push_back(WIFI_DL_LOW_FREQUENCY_0 + (i * 0.3125));
        }
    }
  else if (channelflag % 5 == 1)
    {
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
	  {
		  m_dlSubChannels.push_back(WIFI_DL_LOW_FREQUENCY_1 + (i * 0.3125));
	  }
    }
  else if (channelflag % 5 == 2)
  {
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
	  {
		  m_dlSubChannels.push_back(WIFI_DL_LOW_FREQUENCY_2 + (i * 0.3125));
	  }
  }
  else if (channelflag % 5 == 3)
  {
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
	  {
		  m_dlSubChannels.push_back(WIFI_DL_LOW_FREQUENCY_3 + (i * 0.3125));
	  }
  }
  else if (channelflag % 5 == 4)
  {
	  for (int i = 0; i < RBs_FOR_WIFI; i++)
	  {
		  m_dlSubChannels.push_back(WIFI_DL_LOW_FREQUENCY_4 + (i * 0.3125));
	  }
  }

}


wifi_BandwidthManager::~wifi_BandwidthManager()
{
}

void
wifi_BandwidthManager::SetDLSubChannels(std::vector<double> s)
{
  m_dlSubChannels = s;
}

std::vector<double>
wifi_BandwidthManager::GetDLSubChannels()
{
  return m_dlSubChannels;
}

void
wifi_BandwidthManager::SetDlBandwidth(double b)
{
  m_dlBandwidth = b;
}

double
wifi_BandwidthManager::GetDlBandwidth()
{
	return m_dlBandwidth;
}



int 
wifi_BandwidthManager::Getchannelflag()
{
	return m_channelflag;
}


void
wifi_BandwidthManager::Setchannelflag(int o)
{
	m_channelflag=o;
}



wifi_BandwidthManager*
wifi_BandwidthManager::Copy()
{
	wifi_BandwidthManager *s = new wifi_BandwidthManager();
  s->SetDlBandwidth (GetDlBandwidth ());
  s->Setchannelflag(Getchannelflag());
  s->SetDLSubChannels (GetDLSubChannels ());



  return s;
}

void
wifi_BandwidthManager::Print()
{
  std::cout << 
		  "\t 信道带宽 " << m_dlBandwidth <<
		  "\n\t 信道标志 " << m_channelflag << std::endl;

  std::cout << "\t DL channels: ";
  for (unsigned int i = 0; i < m_dlSubChannels.size(); i++)
    {
	  std::cout << m_dlSubChannels.at (i) << " ";
    }
  std::cout <<  std::endl;

}


