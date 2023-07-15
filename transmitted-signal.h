#ifndef TRANSMITTEDSIGNAL_H_
#define TRANSMITTEDSIGNAL_H_

#include <vector>

class TransmittedSignal {
public:
	TransmittedSignal();
	virtual ~TransmittedSignal();

	void SetValues (std::vector<double> values);
	std::vector<double> Getvalues ();

	TransmittedSignal* Copy ();

private:
	std::vector<double> m_values;  //每一个子载波的传输能量
};

#endif 
