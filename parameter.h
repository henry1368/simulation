
#ifndef PI
#define PI 3.14159265
#endif

#ifndef ap_radius
#define ap_radius 20                   //����apȺ�뾶Ϊ20��
#endif

#ifndef SCENARIO
#define SCENARIO   1               //����(3�ೡ������һ��)     
#endif   

#ifndef SCENARIO_type
#define SCENARIO_type  "TC1"          //���峡��     
#endif 

#ifndef nb_HeNodeB_sector
#define nb_HeNodeB_sector 3            //ÿ��������С��վ����
#endif

#ifndef nb_cell
#define nb_cell 1                     //С������                  
#endif  

#ifndef nb_sector
#define nb_sector 1                   //ÿС����������
#endif 


#ifndef nb_apgrp
#define nb_apgrp 5                     //ÿ��������wifiȺ����
#endif 


#ifndef nb_apingroup
#define nb_apingroup 3                 //ÿ��wifiȺ���ap����
#endif  

#ifndef nb_ue_sector
#define nb_ue_sector 10                //ÿ���������û�����
#endif  

#ifndef nb_totalEnb
#define nb_totalEnb     3              //�ܵĺ��վ����
#endif  


#ifndef nb_totalAP
#define nb_totalAP   6   //�ܵ�AP���� nb_cell*nb_sector*nb_apgrp*nb_apingroup
#endif  

#ifndef nb_totalHenb
#define nb_totalHenb  1     //�ܵ�С��վ����  nb_cell*nb_sector*nb_HeNodeB_sector 
#endif  

#ifndef Ue_Perfloor
#define Ue_Perfloor  5    
#endif 


#ifndef nb_totalUe
#define nb_totalUe   50         //�ܵ��û�����   nb_cell*nb_sector*nb_ue_sector      
#endif  


#ifndef TX_TotalPower
#define TX_TotalPower  46          //���վ�ܷ������� /dBw
#endif 

#ifndef Nt
#define Nt 4                       //��վ\AP����������
#endif  

#ifndef Nr
#define Nr 2                       //�û�����������
#endif   


#ifndef Sim_stop_time
#define Sim_stop_time 50001          //�趨����ʱ��
#endif  

#ifndef FeedBackDelay
#define FeedBackDelay 0               //����ʱ��
#endif



#ifndef Num_Enb_Select
#define Num_Enb_Select   3         //ͳ��sinrʱ��վѡȡ����
#endif  
 
#ifndef Num_Henb_Selec
#define Num_Henb_Select  1
#endif 

