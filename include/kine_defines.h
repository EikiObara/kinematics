// 2017/07/03
// author : eiki obara
#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

#include <vector>
#include "Eigen/Core"

namespace Trl {

typedef Eigen::MatrixXd JointT;

//Joint velocity
typedef Eigen::MatrixXd JointVelT;

//position
typedef Eigen::Vector3d PosT;

//rotation matrix
typedef Eigen::Matrix3d RotMatT;

//hmogeneous translate matrix
typedef Eigen::Matrix4d HtmT;

//center of gravity
typedef Eigen::Vector3d CoGT;

typedef Eigen::Vector3d TorqueT;


//name define with enum
enum JointNameT{
	SHOULDER = 0,	//depend to consentration point of robot arm link.
	ELBOW = 2,
	WRIST = 5,
	FINGER = 6	//ちょっと横着してます。FINGERは７なんだけど。Coordsで使うのに都合よくて
};

enum CoGNameT{
	UPPER = 1,	//depend to link parameter
	FORWARD = 3,	//config the joint which the cog wanted to calculate extends.
	HAND = 6,
};


}	//namespace Trl

#endif // __MY_CONFIG_H__
