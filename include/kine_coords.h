
#ifndef __KINE_COORDS_H__
#define __KINE_COORDS_H__

#include "kine_htm.h"
#include "kine_defines.h"


namespace Trl{

class Coords{
public:
	Coords();
	~Coords();

	void InitCoords(Eigen::MatrixXd &curJointRad);

	void GetFinger(HTM htmObj,PosT &coords);
	void GetWrist (HTM htmObj,PosT &coords);
	void GetElbow (HTM htmObj,PosT &coords);
};

Coords::Coords(){}

Coords::~Coords(){}

void Coords::InitCoords(JointT &curJointRad){
	CalcHTM(curJointRad);
	GetHTMAll();
}

//magic number description(this params can use at 7dof arm only)
// 7 : arm's end effector position
// 6 : arm's wrist position
// 4 : arm's elbow position

void Coords::GetFinger(HTM htmObj,PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[FINGER+1](i,3);
}

void Coords::GetWrist(HTM htmObj,PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[WRIST+1](i,3);
}

void Coords::GetElbow(HTM htmObj,PosT &coords){
	//coords.resize(3);
	for(int i = 0; i < 3; ++i)coords(i) = htm[ELBOW+1](i,3);
}


}	//namespace Trl

#endif //__KINE_COORDS_H__
