
#ifndef __KINE_COORDS_H__
#define __KINE_COORDS_H__

#include "kine_htm.h"
#include <vector>
#include "kine_config.h"

namespace Trl{

class Coords : private HTM{
public:
	Coords(int maxJoint);
	~Coords();

	void InitCoords(Eigen::MatrixXd &curJointRad);


	void GetFinger(Eigen::Vector3d &coords);
	void GetWrist (Eigen::Vector3d &coords);
	void GetElbow (Eigen::Vector3d &coords);
};

Coords::Coords(int maxJoint):HTM(maxJoint){
	SetArmLength(kALength);
	SetOffsetParam(kDLength);
	SetAlphaParam(kAlphaRad);
}

Coords::~Coords(){}

void Coords::InitCoords(Eigen::MatrixXd &curJointRad){
	CalcHTM(curJointRad);
	GetHTMAll();
}

//magic number description(this params can use at 7dof arm only)
// 7 : arm's end effector position
// 6 : arm's wrist position
// 4 : arm's elbow position

void Coords::GetFinger(Eigen::Vector3d &coords){
	coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[7](i,3);
}

void Coords::GetWrist(Eigen::Vector3d &coords){
	coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[6](i,3);
}

void Coords::GetElbow(Eigen::Vector3d &coords){
	coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[3](i,3);
}


}	//namespace Trl

#endif //__KINE_COORDS_H__
