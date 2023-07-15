
#ifndef PACKET_H_
#define PACKET_H_


class Packet {
public:
	Packet();
	virtual ~Packet();

	Packet* Copy();

	void Print ();
/*
	int PDCPHeader_size;         //各层的开销（字节）
	int RLCHeader_size;
	int MACHeader_size;
*/
	double m_overhead;               //开销

	double m_size;                  //有效数据

	bool m_isAFragment;          //是否是分片

	bool m_isTheLatestFragment;  //如果是分片，则是否为最后的一个分片

	int	m_num_application;       //对应该包所对应的业务索引	

	int m_retransmit_number;     //该包的重传次数（默认为0），当重传时会记录重传次数

	bool m_is_FountainCode;      //该包是否为喷泉码的包

};

#endif 
