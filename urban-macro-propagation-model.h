#ifndef URBAN_MACRO_PROPAGATION_MODEL_H_
#define URBAN_MACRO_PROPAGATION_MODEL_H_

#include "lte-propagation-loss-model.h"

class UMa_LtePropagationLossModel : public LtePropagationLossModel {

public:
	UMa_LtePropagationLossModel();
	~UMa_LtePropagationLossModel();
	virtual double Compute_PenetrationLoss(UserEquipment* m_src, NetworkNode* m_dst);      //��͸���
	virtual double Compute_PathLoss(UserEquipment* m_src, NetworkNode* m_dst);             //·��
	virtual double Compute_ShadowFading(UserEquipment* m_src, NetworkNode* m_dst);         //��Ӱ˥��
	virtual double Compute_AntennaGain(UserEquipment* m_src, NetworkNode* m_dst);       //��������
};
#endif