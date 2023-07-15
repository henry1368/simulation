


#include "Mobility.h"
#include "Simulator.h"
#include "position.h"
#include "parameter.h"

Mobility::Mobility(char* str, Positionstate p, int m_move_index)     
{
	/*
	refer to ITU-R M.2135-1 TABLE 8-4
	UMa：30km/h
	UMi: 3km/h
	RMa: 120km/h
	SMa: 90km/h(室外),3km/h(室内)
	移动方向为全向均匀随机
	*/

	//本平台暂不支持用户进行室内和室外之间的切换

	m_Move = m_move_index;
	m_last_update_time = 0.0;
	m_last_update_lte_position = NULL;
	m_last_update_wifi_position = NULL;



	     //第一类场景：目前包括 UMi、UMa、RMa、SMa、TC1
	              if (strcmp(str, "UMi") == 0)
				   {
					   m_speed = 3;
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180; //位置和方向
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

				   if (strcmp(str, "TC1") == 0)        //室内场景
				   {
					   m_speed = 3;
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }

				   if (strcmp(str, "TC2") == 0)        //密集都市信息社会场景
				   {

					   m_speed = (double)rand() / RAND_MAX * 3; //【0，3】km/h
					   m_speedDirection = (double)rand() / RAND_MAX * 360 - 180;
				   }
}




Mobility::~Mobility()
{
  
}


void
Mobility::UpdatePosition(Position* pos)
{
 
  //这个函数根据用户的移动模型计算用户位置更新,并根据;
	double time = Simulator::Init()->Now();
	double x = pos->GetPositionX();
	double y = pos->GetPositionY();

	switch (m_Move)
	{
	case 1:                          //静止
	       {	    break;    }

	case 2:                          //随机移动
	       {
					x += (time - m_last_update_time)*cos(m_speedDirection*PI/180);  //x=x+T*V*cos(a)
					y += (time - m_last_update_time)*sin(m_speedDirection*PI/180);  //y=y+T*V*sin(a)
					pos->SetPositionX(x);
					pos->SetPositionY(y);
					break;
	        }

	case 3:                          //固定轨迹
	       {	
                   //这里应该是直接读取已存储的相关用户位置数据
			        break;    
	       }
	}
   
	m_last_update_time = time;      //更新时间

}


