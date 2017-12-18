#include <iostream>
#include <cmath>

#include "kine_jointTolque.h"

Eigen::Vector4d Vector3d24d(Eigen::Vector3d &buf){
	Eigen::Vector4d ret;
	for(int i = 0; i < 3; ++i)	ret(i,0) = buf(i,0);
	ret(3,0) = 1.0;

	return ret;
}

Eigen::Vector3d Vector4d23d(Eigen::Vector4d &buf){
	Eigen::Vector3d ret;
	for(int i = 0; i < 3; ++i)	ret(i,0) = buf(i,0);

	return ret;
}

void SetJointRad(Trl::JointT &curJointRad){
	curJointRad(0,0) = (90-90) * M_PI / 180;
	curJointRad(1,0) = (90) * M_PI / 180;
	curJointRad(2,0) = (90) * M_PI / 180;
	curJointRad(3,0) = 0 * M_PI / 180;
	curJointRad(4,0) = 0;
	curJointRad(5,0) = -90 * M_PI / 180;
	curJointRad(6,0) = 0;
}

//test cord for kine_jointTolque.h
int main(){
	Trl::TolqueT tolq = Trl::TolqueT::Zero();
	Trl::JointT curJointRad = Trl::JointT(Trl::kMaxJoint,1);

	SetJointRad(curJointRad);

	GetGravityTolque(Trl::SHOULDER,curJointRad,tolq);
	std::cout << tolq << std::endl;

	GetGravityTolque(Trl::ELBOW,curJointRad,tolq);
	std::cout << tolq << std::endl;

	GetGravityTolque(Trl::WRIST,curJointRad,tolq);
	std::cout << tolq << std::endl;


	return 0;
}	


//test cord for kine_centerOfGravity.h
//int main(){
//	Trl::JointT jr = Trl::JointT(Trl::kMaxJoint,1);
//
//	SetJointRad(jr);
//
//	Trl::ArmCoG acg(Trl::kMaxJoint);
//
//	acg.SetLinkParam(Trl::kArmLength, Trl::kOffsetLength, Trl::kAlphaRad);
//	acg.SetLength(Trl::kCoGLength);
//	acg.SetWeight(Trl::kLinkWeight);
//
//	Trl::CoGT cog = Trl::CoGT::Zero();
//
//	std::cout << "SHOULDER\n";
//
//	acg.Get(Trl::SHOULDER, Trl::UPPER, jr, cog);
//	acg.Get(Trl::SHOULDER, Trl::FORWARD, jr, cog);
//	acg.Get(Trl::SHOULDER, Trl::HAND, jr, cog);
//
//	std::cout << "ELBOW\n";
//
//	acg.Get(Trl::ELBOW, Trl::FORWARD, jr, cog);
//	acg.Get(Trl::ELBOW, Trl::HAND, jr, cog);
//	std::cout << "WRIST\n";
//	acg.Get(Trl::WRIST, Trl::HAND, jr, cog);
//}

