#ifndef MADRIDGRID_PROPAGATION_MODEL_H_
#define MADRIDGRID_PROPAGATION_MODEL_H_

#include "lte-propagation-loss-model.h"

class MadridGrid_PropagationLossModel : public LtePropagationLossModel {

public:
	MadridGrid_PropagationLossModel();
	virtual ~MadridGrid_PropagationLossModel();
	virtual void Compute_PowerLoss(UserEquipment* m_src, vector<ENodeB*> *m_dst);
	virtual void Compute_PowerLoss(UserEquipment* m_src, vector<HeNodeB*> *m_dst);

//	virtual double Compute_SmallscaleFading(UserEquipment* m_src, NetworkNode* m_dst);     //С�߶�˥��
	virtual bool Compute_isLOS(UserEquipment* m_src, NetworkNode* m_dst);                  //�Ӿ����
	virtual double Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst);             //·��
	virtual double Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst);      //��͸���
	virtual double Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst);         //��Ӱ˥��
	virtual double Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst);          //��������



};
#endif