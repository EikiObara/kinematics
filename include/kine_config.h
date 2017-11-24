// 2017/07/03
// author : eiki obara
#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

#include <vector>
#include "Eigen/Core"

namespace Trl {
//-----Over all-----
const int kMaxJoint = 7;

const int kMaxLink = 8;

typedef Eigen::MatrixXd JointT;

typedef Eigen::MatrixXd JointVelT;

typedef Eigen::Vector3d TolqueT;

typedef Eigen::Matrix<double,kMaxLink,1> LinkT;

typedef Eigen::Matrix4d RotMatT;	//本当は同時変換行列だけど便宜的に回転行列としています

typedef Eigen::Vector3d CoGT;


//-----Spline-----
const int ROUTE_POINTS = 3;

const int ROUTE_LINK = ROUTE_POINTS - 1;	// link number is defined by nodes. (there is one link between two nodes, so maxLinkVal = maxNodeVal - 1)

const double VIA_LENGTH = 50;		//it uses for CalcViaPos in kine_trajectory.h ... however, it will need not to use. 

// constant numbers 
namespace CHRConfig{
//-----arm length parameter-----

//const double kShoulder	= 164.0;	//肩原点のとき使わない
const double kUpper	= 322.0;
const double kFore	= 257.0;
const double kHand	= 154.0;

const double kArmLen[kMaxLink]= {0.0,0.0,kUpper,0.0,kFore,0.0,0.0,kHand};
const double kOffsetLen[kMaxLink]= {};
const double kAlpha[kMaxLink]= {0.0,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,0.0};

//-----CoG length from center of joint-----
const double CoG_UPPER		= 33.89;	//[mm]
const double CoG_FORWARD	= 142.23;	//[mm]
const double CoG_HAND		= 22.3;	//[mm]
const double kCoGLen[3] = {CoG_UPPER,CoG_FORWARD,CoG_HAND};

//-----  WEIGHT param. -----
const double WEIGHT_UPPER	= 1.00518;
const double WEIGHT_FORWARD	= 0.56333;
const double WEIGHT_HAND	= 0.268;
const double kWeight[3] = {WEIGHT_UPPER,WEIGHT_FORWARD,WEIGHT_HAND};

const double gravityAcceleration = 9806.65;	//m/s^2
const double kGravAccel[3] = {0,-gravityAcceleration, 0};

}	//namespace CHRConfig

//-------------------------------------------------
//----- constant numbers converted eigen type -----
//-------------------------------------------------
const LinkT kArmLength(CHRConfig::kArmLen);

const LinkT kOffsetLength(CHRConfig::kOffsetLen);

const LinkT kAlphaRad(CHRConfig::kAlpha);

const Eigen::Vector3d kCoGLength(CHRConfig::kCoGLen);

const Eigen::Vector3d kLinkWeight(CHRConfig::kWeight);

const Eigen::Vector3d kGravityAccel(CHRConfig::kGravAccel);

}	//namespace Trl
#endif // __MY_CONFIG_H__
