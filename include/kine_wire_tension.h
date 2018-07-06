//2017/12/22
//auther : Eiki obara

#ifndef __KINE_WIRE_TENSION_H__
#define __KINE_WIRE_TENSION_H__

#include "kine_gravityTorque.h"

namespace Trl{

class WireTension{
private:
	GravityTorque gt;

	Eigen::MatrixXd pulley;
public:
	void SetCoGParam(CoGT length, CoGT weight);

	void SetPulleyParam(Eigen::MatrixXd pulley);

	void Get(int jointNum, HTM htmObj, JointT jointRad, double &ret);
};

void WireTension::SetCoGParam(CoGT length, CoGT weight){
	gt.SetCoGParam(length,weight);
}

void WireTension::SetPulleyParam(Eigen::MatrixXd pulley){
	this->pulley = pulley;
}

void WireTension::Get(int jointNum, HTM htmObj,JointT jointRad, double &ret){
	TorqueT retTorq = TorqueT::Zero();

	//関節にかかるトルクを算出
	gt.GetJointTorque(jointNum,htmObj,jointRad,retTorq);

	//std::cout << "pulley\n" << pulley(jointNum) << std::endl;
	//std::cout << "retTorq\n" << retTorq << std::endl;

	//z軸成分のみがロボットの関節角度に影響を及ぼすため、ｚ成分（要素２）を長さで割る
	ret = retTorq(2) / pulley(jointNum);
}

}	//namespace Trl

#endif	//__KINE_WIRE_TENSION_H__
