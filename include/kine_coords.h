
#ifndef __KINE_COORDS_H__
#define __KINE_COORDS_H__

#include "kine_htm.h"
#include <vector>
//#include "kine_config.h"

namespace Trl{

class Coords : private HTM{
public:
	Coords(int maxJoint);
	~Coords();

	void InitCoords(Eigen::MatrixXd &curJointRad);

	void GetFinger(PosT &coords);
	void GetWrist (PosT &coords);
	void GetElbow (PosT &coords);
};

Coords::Coords(int maxJoint):HTM(maxJoint){
	SetArmLength(kArmLength);
	SetOffsetParam(kOffsetLength);
	SetAlphaParam(kAlphaRad);
}

Coords::~Coords(){}

void Coords::InitCoords(JointT &curJointRad){
	CalcHTM(curJointRad);
	GetHTMAll();
}

//magic number description(this params can use at 7dof arm only)
// 7 : arm's end effector position
// 6 : arm's wrist position
// 4 : arm's elbow position

void Coords::GetFinger(PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[7](i,3);
}

void Coords::GetWrist(PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[6](i,3);
}

void Coords::GetElbow(PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[3](i,3);
}


}	//namespace Trl

#endif //__KINE_COORDS_H__
