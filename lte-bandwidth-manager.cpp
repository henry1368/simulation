#include <cstring>
#include "lte-bandwidth-manager.h"
#include <stdio.h>
#include <iostream>
#include "parameter.h"


lte_BandwidthManager::lte_BandwidthManager()
{
}


lte_BandwidthManager::lte_BandwidthManager(char *type)
{
	m_lteBandwidth = LTE_BANDWIDTH;

	m_dlSubChannels.clear();

	if (strcmp(type, "enb") == 0)
	{
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_LTE + (i * 0.18));
		}
	}
	else if (strcmp(type, "henb") == 0)
	{
		for (int i = 0; i < RBs_FOR_LTE; i++)
		{
			m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_LTE_HENB + (i * 0.18));
		}
	}
}


lte_BandwidthManager::~lte_BandwidthManager()
{
}

void
lte_BandwidthManager::SetDLSubChannels(std::vector<double> s)
{
	m_dlSubChannels = s;
}

std::vector<double>
lte_BandwidthManager::GetDLSubChannels()
{
	return m_dlSubChannels;
}

void 
lte_BandwidthManager::SetlteBandwidth(double b)
{
	m_lteBandwidth = b;
}

double 
lte_BandwidthManager::GetlteBandwidth()
{
	return m_lteBandwidth;
}

lte_BandwidthManager*
lte_BandwidthManager::Copy()
{
	lte_BandwidthManager *s = new lte_BandwidthManager();
	s->SetlteBandwidth(GetlteBandwidth());
	return s;
}

void
lte_BandwidthManager::Print()
{
	std::cout << "lte_BandwidthManager: "<< std::endl;

	std::cout << "\n\t m_lteBandwidth " << m_lteBandwidth;
		
	std::cout << "\t  lte-channels: ";
	for (unsigned int i = 0; i < m_dlSubChannels.size(); i++)
	{
		std::cout << m_dlSubChannels.at(i) << " ";
	}
	std::cout << std::endl;

}


