//2017/12/22
//auther : eiki obara

#ifndef __KINE_CONSTS_H__
#define __KINE_CONSTS_H__

#include "Eigen/Core"

namespace Trl{

//spline
const int ROUTE_POINTS = 3;

//spline link number is defined by nodes.
//(there is one link between two nodes,so maxLinkVal = maxNodeVal-1)
const int ROUTE_LINK = ROUTE_POINTS - 1;

// via points constant number. maybe no need in near future
//use in CalcViaPos in kine_trajectory.h
const double VIA_LENGTH = 50.0;

//dynamic constants
const double kGravityAcceleration = 9806.65;	//mm/s^2

const double kGravAccel[3] = {0, -kGravityAcceleration, 0};

Eigen::Vector3d kGravityAccel(kGravAccel);

const double kThickSpringConst = 100;
const double kThinSpringConst = 50;

const double kSpring[2] = {kThickSpringConst,kThinSpringConst};

Eigen::Vector2d kSpringConst(kSpring);

}	//namespace Trl

#endif //__KINE_CONSTS_H__

