// 2017/07/03
// author : eiki obara
#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

#include <vector>
#include "Eigen/Core"

namespace Trl {
//-----Over all-----
typedef Eigen::MatrixXd JointT;

typedef Eigen::MatrixXd JointVelT;

typedef Eigen::Vector3d TorqueT;

typedef Eigen::Matrix3d RotMatT;

typedef Eigen::Matrix4d HtmT;

typedef Eigen::Vector3d CoGT;

typedef Eigen::Vector3d PosT;

//-----Spline-----
const int ROUTE_POINTS = 3;

const int ROUTE_LINK = ROUTE_POINTS - 1;	// link number is defined by nodes. (there is one link between two nodes, so maxLinkVal = maxNodeVal - 1)

const double VIA_LENGTH = 50;		//it uses for CalcViaPos in kine_trajectory.h ... however, it will need not to use. 

enum JointNameT{
	SHOULDER = 0,	//depend to consentration point of robot arm link.
	ELBOW = 2,
	WRIST = 5,
};

enum CoGNameT{
	UPPER = 1,	//depend to link parameter
	FORWARD = 3,	//config the joint which the cog wanted to calculate extends.
	HAND = 6,
};


}	//namespace Trl

#endif // __MY_CONFIG_H__
