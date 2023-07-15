


#ifndef WIFICHANNEL_H_
#define WIFICHANNEL_H_

#include <vector>

class NetworkNode;
class TransmittedSignal;
class PacketBurst;                       //���Ķ���


class WifiChannel {
public:
	WifiChannel();
	virtual ~WifiChannel();

	void StartTx(PacketBurst* p, TransmittedSignal* txSignal, NetworkNode* src);
	void StartRx(PacketBurst* p, TransmittedSignal* psd, NetworkNode* src);

	void AddDevice(NetworkNode* d);
	void DelDevice(NetworkNode* d);
	bool IsAttached(NetworkNode* d);

	std::vector<NetworkNode*>* GetDevices(void);

	void SetChannelId(int id);
	int GetChannelId(void);


private:
	std::vector<NetworkNode*> *m_attachedDevices;                //���ӵ�

	int m_channelId;
};

#endif
