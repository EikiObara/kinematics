//2017/12/22
//auther : Eiki obara

#ifndef __KINE_WIRE_TENSION_H__
#define __KINE_WIRE_TENSION_H__

#include "kine_gravityTorque.h"

//トルクをもらう
//トルクのZ軸成分だけ抽出
//トルクをプーリ半径で割る

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

	gt.GetJointTorque(jointNum,htmObj,jointRad,retTorq);

	//std::cout << "pulley\n" << pulley(jointNum) << std::endl;
	//std::cout << "retTorq\n" << retTorq << std::endl;

	//z軸成分のみがロボットの関節角度に影響を及ぼす。
	ret = retTorq(2) / pulley(jointNum);
}

}	//namespace Trl

#endif	//__KINE_WIRE_TENSION_H__
