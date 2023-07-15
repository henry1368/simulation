/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* Copyright (c) 2010,2011,2012 TELEMATICS LAB, Politecnico di Bari
*
* This file is part of LTE-Sim
*
* LTE-Sim is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation;
*
* LTE-Sim is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
*
* Author: Giuseppe Piro <g.piro@poliba.it>
*/

#ifndef WIFI_CQIMANAGER_H_
#define WIFI_CQIMANAGER_H_

#include <vector>

class NetworkNode;

class wifi_cqiManager {
public:

	wifi_cqiManager();
	virtual ~wifi_cqiManager();

	void SetDevice(NetworkNode* d);
	NetworkNode* GetDevice(void);

	void SetSendCqi(bool b);
	bool GetSendCqi(void);

	void SetReportingInterval(int i);
	int GetReportingInterval(void);

	void SetLastSent();
	long int GetLastSent(void);

	virtual void CreateCqiFeedbacks(std::vector<double> sinr) = 0;

	bool NeedToSendFeedbacks(void);

private:

	bool m_sendCqi;   

	int m_reportingInterval;
	long int m_lastSent;

	NetworkNode* m_device;

};

#endif /* CQIMANAGER_H_ */
