// 2017/07/03
// author : eiki obara
#include <cmath>
#include "Eigen/Core"
#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__
#include <cmath>
#include "Eigen/Core"
namespace Trl {
//-----Over all-----
static const int kMaxJoint = 8;

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

const double kALen[kMaxJoint]		= {0.0,0.0,kUpper,0.0,kFore,0.0,0.0,kHand};
const double kDLen[kMaxJoint]		= {};
const double kAlpha[kMaxJoint]	= {0.0,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,-M_PI/2,M_PI/2,0.0};
}	//namespace CHRConfig

const Eigen::Matrix<double,kMaxJoint,1> kALength(CHRConfig::kALen);
const Eigen::Matrix<double,kMaxJoint,1> kDLength(CHRConfig::kDLen);
const Eigen::Matrix<double,kMaxJoint,1> kAlphaRad(CHRConfig::kAlpha);

}	//namespace Trl
#endif // __MY_CONFIG_H__
