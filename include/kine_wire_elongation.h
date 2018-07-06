//2017/12/22
//auther : eiki obara

#ifndef __KINE_WIRE_ELONGATION_H__
#define __KINE_WIRE_ELONGATION_H__

#include "kine_wireTension.h"
#include "kine_math_consts.h"

namespace Trl{

class WireElongation{
private:
	WireTension wt;
public:
	void SetParam(CoGT length,CoGT weight,Eigen::MatrixXd pulley);

	void Get(int jointNum,HTM htmObj, JointT jointRad, double &ret);
};

void WireElongation::SetParam(CoGT length,CoGT weight,Eigen::MatrixXd pulley){
	wt.SetCoGParam(length,weight);
	wt.SetPulleyParam(pulley);
}

void WireElongation::Get(int jointNum,HTM htmObj,JointT jointRad,double &ret){
	double tension = 0;

	wt.Get(jointNum,htmObj,jointRad,tension);

	std::cout << "tension->" << tension << std::endl;

	if(0 <= jointNum && jointNum <= 2){
		ret = tension/kSpringConst(0);
	}else if(3 <= jointNum && jointNum <= 6){
		ret = tension/kSpringConst(1);
	}
	std::cout << "elongation->" << ret << std::endl;
}

}	//namespace Trl

#endif //__KINE_WIRE_ELONGATION_H__
