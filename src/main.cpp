#include <iostream>
#include "kine_gravityCompensation.h"
#include "kine_config.h"

int main(void){
	//定数　＝　質量、重力加速度
	//変数　＝　重心位置、角度、

	Trl::JointT curJointRad;
	curJointRad = Trl::JointT(Trl::kMaxJoint,1);
	Trl::CoGT CoG;

	curJointRad(0,0) = (50-90) * M_PI / 180;
	curJointRad(1,0) = (90) * M_PI / 180;
	curJointRad(2,0) = (90) * M_PI / 180;
	curJointRad(3,0) = 90 * M_PI / 180;
	curJointRad(4,0) = 0;
	curJointRad(5,0) = -50 * M_PI / 180;
	curJointRad(6,0) = 0;

	Trl::ArmCoG ac(Trl::kMaxJoint);

	ac.SetLinkParam(Trl::kALength,Trl::kDLength,Trl::kAlphaRad);

	ac.SetCoGParam(Trl::kCoG[0],Trl::kCoG[1],Trl::kCoG[2]);

	ac.GetCoGUpper(curJointRad, CoG);
	ac.GetCoGForward(curJointRad, CoG);
	ac.GetCoGHand(curJointRad, CoG);
	
	return 0;
}
