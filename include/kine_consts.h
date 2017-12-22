//2017/12/22
//auther : eiki obara

#ifndef __KINE_CONSTS_H__
#define __KINE_CONSTS_H__

#include "Eigen/Core"

//spline
const int ROUTE_POINTS = 3;

//spline link number is defined by nodes.
//(there is one link between two nodes,so maxLinkVal = maxNodeVal-1)
const int ROUTE_LINK = ROUTE_POINTS - 1;

// via points constant number. maybe no need in near future
//use in CalcViaPos in kine_trajectory.h
const double VIA_LENGTH = 50.0;

//dynamic constants
const double gravityAcceleration = 9806.65;	//mm/s^2

const double kGravAccel[3] = {0, -gravityAcceleration, 0};

Eigen::Vector3d kGravityAccel(kGravAccel);

#endif //__KINE_CONSTS_H__

