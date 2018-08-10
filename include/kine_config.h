// 2017/07/03
// author : eiki obara
#include <cmath>
#include "Eigen/Core"
#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__
#include <cmath>
#include <vector>

#include "Eigen/Core"
namespace Trl {
//-----Over all-----
static const int kMaxJoint = 6;

//-----Spline-----
const int ROUTE_POINTS = 3;

const int ROUTE_LINK = ROUTE_POINTS - 1;

const double VIA_LENGTH = 50;

namespace CHRConfig{
//-----arm parameter-----

const int kLinkNum = kMaxJoint + 1;

//link length[m]
const double kShoulder	= 0.050;	//肩原点のとき使わない
const double kUpper	= 0.500;
const double kFore	= 0.500;
const double kHand	= 0.100;

//link offset
const double kUpperOffset = 0.0;
const double kForeOffset = 0.0;
const double kHandOffset = 0.0;

//PUMA
const double kALen[kLinkNum]	= {0.0,	0.0,		kUpper,	0.0,		0.0,		0.0,		0.0};
const double kAlpha[kLinkNum]	= {0.0,	-M_PI/2,	0.0,	-M_PI/2,	M_PI/2,	-M_PI/2,	0.0};
const double kDLen[kLinkNum]	= {0.0,	0.0,		0.0,		kFore,	0.0,	0.0,	kHand};

//stanford manipulator
//const double kALen[kLinkNum]	= {0.0,	0.0,		0.0,		0.0,		0.0,		0.0,	kHandOffset};
//const double kAlpha[kLinkNum]	= {0.0,	-M_PI/2,	M_PI/2,		0.0,	-M_PI/2,	M_PI/2,	0.0};
//const double kDLen[kLinkNum]	= {0.0,	kUpper,		kFore,		0.0,		0.0,		0.0,	kHand};

}	//namespace CHRConfig

const Eigen::Matrix<double,CHRConfig::kLinkNum,1> kALength(CHRConfig::kALen);
const Eigen::Matrix<double,CHRConfig::kLinkNum,1> kDLength(CHRConfig::kDLen);
const Eigen::Matrix<double,CHRConfig::kLinkNum,1> kAlphaRad(CHRConfig::kAlpha);


}	//namespace Trl
#endif // __MY_CONFIG_H__
