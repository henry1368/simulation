#ifndef DCF_STATE_H
#define DCF_STATE_H


#define dot11LongRetryLimit  4           
#define dot11ShortRetryLimit 7          
      //背景     
#define BK_CwMin  31                  //最小退避时隙
#define BK_CwMax 1023                //最大退避时隙             
#define BK_AIFSN  7                  //额外等待时隙数
      //尽力而为
#define BE_CwMin  31                  //最小退避时隙
#define BE_CwMax 1023                //最大退避时隙             
#define BE_AIFSN  3                  //额外等待时隙数
      //视频
#define VI_CwMin  15                   //最小退避时隙
#define VI_CwMax  31                   //最大退避时隙             
#define VI_AIFSN  2                   //额外等待时隙数
       //语音
#define VO_CwMin  7                   //最小退避时隙
#define VO_CwMax  15                  //最大退避时隙             
#define VO_AIFSN  2                  //额外等待时隙数


#define SlotTime    9                 //us
#define SIFS       16                 //us
#define DIFS       34                 //us
#define ACKTimeout 58                 //us



#include <stdint.h>
class NetworkNode;
/*
此类用于记录用户的退避情况，用于信道竞争时
*/
class DcfState
{
public:

DcfState();
virtual ~DcfState();

void UpdateFailedState();              //发送失败时退避情况的更新

void ResetState();                     //用户在发包成功或初始化时根据业务重新更新退避状态  

bool IsAccessRequested() const;    //返回是否进行信道接入竞争

void NotifyAccessRequested();      //告知该用户是否进行信道接入竞争

uint32_t m_SRC;               //short retry count
uint32_t m_LRC;               //long retry count


double m_aifsn;               //不同业务等待时长不一样   AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime  AIFSN(AC)为不同业务的额外等待时间
uint32_t m_backoffSlots;      //剩余退避时隙
double  m_totalbackofftime;    //总共退避时间：AIFS+backoffSlot*aSlotTime
uint32_t m_cw;                //退避区间
uint32_t m_cw_max;             //最大退避区间
bool m_accessRequested;       //是否想接入信道
double m_backoffStart;        //退避开始时刻或者上次退避计时器更新时刻
double m_accesstime;          //接入时间
NetworkNode * Belong_Node;    //所属节点

bool Is_initial;             //用户是否已经根据业务初始化

};

#endif