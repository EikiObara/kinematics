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
typedef Eigen::MatrixXd TolqueT;
typedef Eigen::Matrix<double,kMaxLink,1> LinkT;

typedef Eigen::Matrix4d RotMatT;

typedef Eigen::Matrix<double,3,1> CoGT;

//-----Spline-----
const int ROUTE_POINTS = 3;

const int ROUTE_LINK = ROUTE_POINTS - 1;

const double VIA_LENGTH = 50;

namespace CHRConfig{
//-----arm parameter-----

//const double kShoulder	= 164.0;	//肩原点のとき使わない
const double kUpper	= 322.0;
const double kFore	= 257.0;
const double kHand	= 154.0;

const double kALen[kMaxLink]= {0.0,0.0,kUpper,0.0,kFore,0.0,0.0,kHand};
const double kDLen[kMaxLink]= {};
const double kAlpha[kMaxLink]= {0.0,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,0.0};

}	//namespace CHRConfig

const LinkT kALength(CHRConfig::kALen);
const LinkT kDLength(CHRConfig::kDLen);
const LinkT kAlphaRad(CHRConfig::kAlpha);

const double CoG_UPPER		= 33.89;
const double CoG_FORWARD	= 142.23;
const double CoG_HAND		= 22.3;

const double kCoG[3] = {CoG_UPPER,CoG_FORWARD,CoG_HAND};

const double WEIGHT_UPPER	= 1005.18;
const double WEIGHT_FORWARD	= 563.33;
const double WEIGHT_HAND	= 0.268;

const double kWeight[3] = {WEIGHT_UPPER,WEIGHT_FORWARD,WEIGHT_HAND};
}	//namespace Trl
#endif // __MY_CONFIG_H__
