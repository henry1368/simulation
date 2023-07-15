


#include "Mobility.h"
#include "Simulator.h"
#include "position.h"
#include "parameter.h"

Mobility::Mobility(char* str, Positionstate p, int m_move_index)     
{
	/*
	refer to ITU-R M.2135-1 TABLE 8-4
	UMa��30km/h
	UMi: 3km/h
	RMa: 120km/h
	SMa: 90km/h(����),3km/h(����)
	�ƶ�����Ϊȫ��������
	*/

	//��ƽ̨�ݲ�֧���û��������ں�����֮����л�

	m_Move = m_move_index;
	m_last_update_time = 0.0;
	m_last_update_lte_position = NULL;
	m_last_update_wifi_position = NULL;



	     //��һ�ೡ����Ŀǰ���� UMi��UMa��RMa��SMa��TC1
	              if (strcmp(str, "UMi") == 0)
				   {
					   m_speed = 3;
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180; //λ�úͷ���
				   }

				   if (strcmp(str, "UMa") == 0)
				   {         
					   m_speed = 30;                        //100% of users outdoors in vehicles
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }

				   if (strcmp(str, "RMa") == 0)
				   {				     
					   m_speed = 120;                        //100% of users outdoors in high speed vehicles
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }

				   if (strcmp(str, "SMa") == 0)
				   {
					   double Rn = (double)rand() / RAND_MAX;    //50% users vehicles and 50% of users indoors
					   if (p== m_Vehicle)			            
						   m_speed = 90;
					   else if (p == m_Indoor)	
						   m_speed = 3;

					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }

				   if (strcmp(str, "TC1") == 0)        //���ڳ���
				   {
					   m_speed = 3;
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }

				   if (strcmp(str, "TC2") == 0)        //�ܼ�������Ϣ��᳡��
				   {

					   m_speed = (double)rand() / RAND_MAX * 3; //��0��3��km/h
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }
}




Mobility::~Mobility()
{
  
}


void
Mobility::UpdatePosition(Position* pos)
{
 
  //������������û����ƶ�ģ�ͼ����û�λ�ø���,������;
	double time = Simulator::Init()->Now();
	double x = pos->GetPositionX();
	double y = pos->GetPositionY();

	switch (m_Move)
	{
	case 1:                          //��ֹ
	       {	    break;    }

	case 2:                          //����ƶ�
	       {
					x += (time - m_last_update_time)*cos(m_speedDirection*PI/180);  //x=x+T*V*cos(a)
					y += (time - m_last_update_time)*sin(m_speedDirection*PI/180);  //y=y+T*V*sin(a)
					pos->SetPositionX(x);
					pos->SetPositionY(y);
					break;
	        }

	case 3:                          //�̶��켣
	       {	
                   //����Ӧ����ֱ�Ӷ�ȡ�Ѵ洢������û�λ������
			        break;    
	       }
	}
   
	m_last_update_time = time;      //����ʱ��

}


