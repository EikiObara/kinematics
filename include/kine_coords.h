
#ifndef __KINE_COORDS_H__
#define __KINE_COORDS_H__

#include <vector>
#include "kine_htm.h"
#include "kine_config.h"

namespace Trl{

class Coords{
private:
	HTM htmObj;

	void InitCoords(Eigen::MatrixXd &curJointRad);
public:
	Coords(int maxJoint, Eigen::MatrixXd aLength, Eigen::MatrixXd dLength, Eigen::MatrixXd alpha);
	~Coords();

	Eigen::Vector3d GetFinger(Eigen::MatrixXd &jointRad);
	Eigen::Vector3d GetWrist(Eigen::MatrixXd &jointRad);
	Eigen::Vector3d GetElbow(Eigen::MatrixXd &jointRad);
};

Coords::Coords(int maxJoint, Eigen::MatrixXd aLength, Eigen::MatrixXd dLength, Eigen::MatrixXd alpha):htmObj(maxJoint){
	htmObj.SetArmLength(aLength);
	htmObj.SetOffsetParam(dLength);
	htmObj.SetAlphaParam(alpha);
}

Coords::~Coords(){}

void Coords::InitCoords(Eigen::MatrixXd &curJointRad){
	htmObj.CalcHTM(curJointRad);
	htmObj.GetHTMAll();
}

//magic number description(this params can use at 7dof arm only)
// 7 : arm's end effector position
// 6 : arm's wrist position
// 4 : arm's elbow position

Eigen::Vector3d Coords::GetFinger(Eigen::MatrixXd &jointRad){
	InitCoords(jointRad);
	Eigen::Vector3d coords;
	for(int i = 0; i < 3; ++i)coords(i) = htmObj.htm[6](i,3);
	return coords;
}

Eigen::Vector3d Coords::GetWrist(Eigen::MatrixXd &jointRad){
	InitCoords(jointRad);
	Eigen::Vector3d coords;
	for(int i = 0; i < 3; ++i)coords(i) = htmObj.htm[5](i,3);
	return coords;
}

Eigen::Vector3d Coords::GetElbow(Eigen::MatrixXd &jointRad){
	InitCoords(jointRad);
	Eigen::Vector3d coords;
	for(int i = 0; i < 3; ++i)coords(i) = htmObj.htm[2](i,3);
	return coords;
}

}	//namespace Trl

#endif //__KINE_COORDS_H__
