#ifndef DCF_STATE_H
#define DCF_STATE_H


#define dot11LongRetryLimit  4           
#define dot11ShortRetryLimit 7          
      //����     
#define BK_CwMin  31                  //��С�˱�ʱ϶
#define BK_CwMax 1023                //����˱�ʱ϶             
#define BK_AIFSN  7                  //����ȴ�ʱ϶��
      //������Ϊ
#define BE_CwMin  31                  //��С�˱�ʱ϶
#define BE_CwMax 1023                //����˱�ʱ϶             
#define BE_AIFSN  3                  //����ȴ�ʱ϶��
      //��Ƶ
#define VI_CwMin  15                   //��С�˱�ʱ϶
#define VI_CwMax  31                   //����˱�ʱ϶             
#define VI_AIFSN  2                   //����ȴ�ʱ϶��
       //����
#define VO_CwMin  7                   //��С�˱�ʱ϶
#define VO_CwMax  15                  //����˱�ʱ϶             
#define VO_AIFSN  2                  //����ȴ�ʱ϶��


#define SlotTime    9                 //us
#define SIFS       16                 //us
#define DIFS       34                 //us
#define ACKTimeout 58                 //us



#include <stdint.h>
class NetworkNode;
/*
�������ڼ�¼�û����˱�����������ŵ�����ʱ
*/
class DcfState
{
public:

DcfState();
virtual ~DcfState();

void UpdateFailedState();              //����ʧ��ʱ�˱�����ĸ���

void ResetState();                     //�û��ڷ����ɹ����ʼ��ʱ����ҵ�����¸����˱�״̬  

bool IsAccessRequested() const;    //�����Ƿ�����ŵ����뾺��

void NotifyAccessRequested();      //��֪���û��Ƿ�����ŵ����뾺��

uint32_t m_SRC;               //short retry count
uint32_t m_LRC;               //long retry count


double m_aifsn;               //��ͬҵ��ȴ�ʱ����һ��   AIFS[AC]=aSIFITime+AIFSN(AC)*aSlotTime  AIFSN(AC)Ϊ��ͬҵ��Ķ���ȴ�ʱ��
uint32_t m_backoffSlots;      //ʣ���˱�ʱ϶
double  m_totalbackofftime;    //�ܹ��˱�ʱ�䣺AIFS+backoffSlot*aSlotTime
uint32_t m_cw;                //�˱�����
uint32_t m_cw_max;             //����˱�����
bool m_accessRequested;       //�Ƿ�������ŵ�
double m_backoffStart;        //�˱ܿ�ʼʱ�̻����ϴ��˱ܼ�ʱ������ʱ��
double m_accesstime;          //����ʱ��
NetworkNode * Belong_Node;    //�����ڵ�

bool Is_initial;             //�û��Ƿ��Ѿ�����ҵ���ʼ��

};

#endif