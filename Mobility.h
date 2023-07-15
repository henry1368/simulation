
#ifndef MOBILITY_H_
#define MOBILITY_H_

//�ƶ�ģ�Ͷ������ڲ�ͬ�����µ��û����ƶ�ģ�ͣ���ص��ƶ��ٶȡ������λ�ø��º�����
/*
�趨�û����ƶ�ģ�͹�3�֣�
1.��ֹ״̬
2.����ƶ�
3.�̶��켣

*/
//���е�2���ƶ�ģ������M2135.Table8-4 ��ع涨�����û����ƶ��ٶȺͷ���

#include <stdlib.h>
#include"position.h"


class Mobility {                                 
public:
	Mobility(char* str, Positionstate p,int m_move_index);  //�������ƶ�ģ��
	virtual ~Mobility();

	int m_Move;                            //�ƶ�ģ��   (��ֹ�����������)
	double m_speed;			 		       //�ٶ� Km/h
	double m_speedDirection;               //�ƶ�����

	double m_last_update_time;                //�ϴθ���λ��ʱ��
	Position* m_last_update_lte_position;     //LTE���ϴη�����߶ȱ仯��λ��
	Position* m_last_update_wifi_position;    //WIFI���ϴη�����߶ȱ仯��λ��

	void UpdatePosition(Position* pos);       


};

#endif 
