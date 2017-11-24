#include <iostream>
#include <cmath>

#include "kine_centerOfGravity.h"
#include "kine_config.h"
#include "kine_htm.h"

Eigen::Vector4d Vector3d24d(Eigen::Vector3d &buf){
	Eigen::Vector4d ret;
	for(int i = 0; i < 3; ++i){
		ret(i,0) = buf(i,0);
	}
	ret(3,0) = 1.0;

	return ret;
}

Eigen::Vector3d Vector4d23d(Eigen::Vector4d &buf){
	Eigen::Vector3d ret;
	for(int i = 0; i < 3; ++i){
		ret(i,0) = buf(i,0);
	}

	return ret;
}

void SetJointRad(Trl::JointT &curJointRad){
	curJointRad(0,0) = (50-90) * M_PI / 180;
	curJointRad(1,0) = (90) * M_PI / 180;
	curJointRad(2,0) = (90) * M_PI / 180;
	curJointRad(3,0) = 90 * M_PI / 180;
	curJointRad(4,0) = 0;
	curJointRad(5,0) = -50 * M_PI / 180;
	curJointRad(6,0) = 0;
}

int main(){
	Trl::JointT jr = Trl::JointT(Trl::kMaxJoint,1);

	SetJointRad(jr);

	Trl::ArmCoG acg(Trl::kMaxJoint);

	acg.SetLinkParam(Trl::kArmLength, Trl::kOffsetLength, Trl::kAlphaRad);
	acg.SetLength(Trl::kCoGLength);
	acg.SetWeight(Trl::kLinkWeight);

	Trl::CoGT cog = Trl::CoGT::Zero();

	acg.Get(Trl::shoulder, Trl::forward, jr, cog);

	std::cout << "cog\n " << cog << std::endl;
}

//int main(void){
//	//定数　＝　質量、重力加速度
//	//変数　＝　重心位置、角度、
//
//	Trl::JointT curJointRad = Trl::JointT(Trl::kMaxJoint,1);
//
//	SetJointRad(curJointRad);
//
//	Trl::CoGT CoG[6];
//
//	Trl::TolqueT tolqOrigin = Trl::TolqueT::Zero();
//	Trl::TolqueT tolq[Trl::kMaxJoint];
//
//	Trl::ArmCoG ac(Trl::kMaxJoint);
//
//	ac.SetLinkParam(Trl::kArmLength,Trl::kOffsetLength,Trl::kAlphaRad);
//
//	ac.SetLength(Trl::kCoGLength);
//	ac.SetWeight(Trl::kLinkWeight);
//
//	ac.GetFromShoulder(Trl::upper,curJointRad, CoG[0]);
//	ac.GetFromShoulder(Trl::forward,curJointRad, CoG[1]);
//	ac.GetFromShoulder(Trl::hand,curJointRad, CoG[2]);
//
//	for(int i = 0; i < 3; ++i){
//		Eigen::Vector3d gravityForce;
//
//		gravityForce = Trl::kGravityAccel * Trl::kLinkWeight(i);
//		
//		tolqOrigin = tolqOrigin + CoG[i].cross(gravityForce);
//	}
//
//	//std::cout << "tolque original\n"<< tolqOrigin << std::endl;
//
//	Trl::HTM htmObj(7);
//
//	htmObj.SetArmLength(Trl::kArmLength);
//	htmObj.SetOffsetParam(Trl::kOffsetLength);
//	htmObj.SetAlphaParam(Trl::kAlphaRad);
//
//	htmObj.CalcHTM(curJointRad);
//	htmObj.GetHTMAll();
//
//	for(int i = 0; i < htmObj.htm.size(); ++i)std::cout << htmObj.htm[i] << std::endl;
//
//	Eigen::Vector4d cogInterface[7];
//
//	for(int i = 0; i < 2; ++i)	cogInterface[i] = Vector3d24d(CoG[0]);
//	for(int i = 2; i < 3; ++i)	cogInterface[i] = Vector3d24d(CoG[1]);
//	for(int i = 3; i < 7; ++i)	cogInterface[i] = Vector3d24d(CoG[2]);
//
//	for(int i = 0; i < 7; ++i){
//		Trl::RotMatT buf;
//		htmObj.GetHTM(0,i,buf);
//
//		Eigen::Vector4d mul;
//
//		//std::cout << buf.inverse() << std::endl;
//
//		mul = buf.inverse() * Vector3d24d(tolqOrigin);
//
//		tolq[i] = Vector4d23d(mul);
//
//		//std::cout << i << "\n"  << tolq[i] << std::endl;
//	}
//
//	
//	return 0;
//}
