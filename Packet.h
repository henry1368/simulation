
#ifndef PACKET_H_
#define PACKET_H_


class Packet {
public:
	Packet();
	virtual ~Packet();

	Packet* Copy();

	void Print ();
/*
	int PDCPHeader_size;         //����Ŀ������ֽڣ�
	int RLCHeader_size;
	int MACHeader_size;
*/
	double m_overhead;               //����

	double m_size;                  //��Ч����

	bool m_isAFragment;          //�Ƿ��Ƿ�Ƭ

	bool m_isTheLatestFragment;  //����Ƿ�Ƭ�����Ƿ�Ϊ����һ����Ƭ

	int	m_num_application;       //��Ӧ�ð�����Ӧ��ҵ������	

	int m_retransmit_number;     //�ð����ش�������Ĭ��Ϊ0�������ش�ʱ���¼�ش�����

	bool m_is_FountainCode;      //�ð��Ƿ�Ϊ��Ȫ��İ�

};

#endif 
