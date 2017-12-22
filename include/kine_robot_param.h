// kine_robot_param.h
// 2017/12/18
// auther : eiki Obara

#ifndef __KINE_ROBOT_PARAM_H__
#define __KINE_ROBOT_PARAM_H__

#include "Eigen/Core"

namespace Trl{

const int kMaxJoint = 7;

const int kMaxLink = 8;

//typedef
typedef Eigen::Matrix<double,kMaxLink,1> LinkT;

typedef Eigen::Matrix<double,kMaxLink,1> PulleyT;

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
const double CoG_HAND		= 22.3;		//[mm]
const double kCoGLen[3] = {CoG_UPPER,CoG_FORWARD,CoG_HAND};

//-----  WEIGHT param. -----
const double WEIGHT_UPPER	= 1.00518;	//kg
const double WEIGHT_FORWARD	= 0.56333;	//kg
const double WEIGHT_HAND	= 0.268;	//kg
const double kWeight[3] = {WEIGHT_UPPER,WEIGHT_FORWARD,WEIGHT_HAND};

//pulley radius
const double kPulleyRadius[kMaxLink] = {48.5, 31, 31, 21, 19.5, 13, 13.5, 0};

}	//namespace CHRConfig

//----- constant numbers converted eigen type -----

LinkT kArmLength(CHRConfig::kArmLen);

LinkT kOffsetLength(CHRConfig::kOffsetLen);

LinkT kAlphaRad(CHRConfig::kAlpha);

Eigen::Vector3d kCoGLength(CHRConfig::kCoGLen);

Eigen::Vector3d kLinkWeight(CHRConfig::kWeight);

PulleyT kPulleyRadius(CHRConfig::kPulleyRadius);

}	//namespace Trl

#endif //__KINE_ROBOT_PARAM_H__
