
#ifndef PI
#define PI 3.14159265
#endif

#ifndef ap_radius
#define ap_radius 20                   //定义ap群半径为20米
#endif

#ifndef SCENARIO
#define SCENARIO   1               //场景(3类场景中哪一类)     
#endif   

#ifndef SCENARIO_type
#define SCENARIO_type  "TC1"          //具体场景     
#endif 

#ifndef nb_HeNodeB_sector
#define nb_HeNodeB_sector 3            //每个扇区的小基站数量
#endif

#ifndef nb_cell
#define nb_cell 1                     //小区个数                  
#endif  

#ifndef nb_sector
#define nb_sector 1                   //每小区扇区个数
#endif 


#ifndef nb_apgrp
#define nb_apgrp 5                     //每个扇区的wifi群个数
#endif 


#ifndef nb_apingroup
#define nb_apingroup 3                 //每个wifi群里的ap个数
#endif  

#ifndef nb_ue_sector
#define nb_ue_sector 10                //每个扇区的用户个数
#endif  

#ifndef nb_totalEnb
#define nb_totalEnb     3              //总的宏基站个数
#endif  


#ifndef nb_totalAP
#define nb_totalAP   6   //总的AP个数 nb_cell*nb_sector*nb_apgrp*nb_apingroup
#endif  

#ifndef nb_totalHenb
#define nb_totalHenb  1     //总的小基站个数  nb_cell*nb_sector*nb_HeNodeB_sector 
#endif  

#ifndef Ue_Perfloor
#define Ue_Perfloor  5    
#endif 


#ifndef nb_totalUe
#define nb_totalUe   50         //总的用户个数   nb_cell*nb_sector*nb_ue_sector      
#endif  


#ifndef TX_TotalPower
#define TX_TotalPower  46          //宏基站总发射能量 /dBw
#endif 

#ifndef Nt
#define Nt 4                       //基站\AP发射天线数
#endif  

#ifndef Nr
#define Nr 2                       //用户接收天线数
#endif   


#ifndef Sim_stop_time
#define Sim_stop_time 50001          //设定仿真时间
#endif  

#ifndef FeedBackDelay
#define FeedBackDelay 0               //反馈时延
#endif



#ifndef Num_Enb_Select
#define Num_Enb_Select   3         //统计sinr时基站选取个数
#endif  
 
#ifndef Num_Henb_Selec
#define Num_Henb_Select  1
#endif 

