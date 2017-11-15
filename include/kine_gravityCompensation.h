#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "kine_htm.h"
#include "kine_config.h"

namespace Trl{

class ArmCoG : private HTM{
private:
	double kUpper;
	double kForward;
	double kHand;

	LinkT alpha;

	RotMatT upperCoG;
	RotMatT forwardCoG;
	RotMatT handCoG;

public:
	ArmCoG(double maxLink);

	void SetLinkParam(const LinkT armLength, const LinkT offset, const LinkT distortion);
	void SetCoGParam(const double upper, const double forward, const double hand);

	void GetCoGUpper(JointT &curJointRad, CoGT &cog);
	void GetCoGForward(JointT &curJointRad, CoGT &cog);
	void GetCoGHand(JointT &curJointRad, CoGT &cog);
};

ArmCoG::ArmCoG(double maxLink) : HTM(maxLink){
	upperCoG	= Eigen::Matrix4d::Zero();
	forwardCoG	= Eigen::Matrix4d::Zero();
	handCoG		= Eigen::Matrix4d::Zero();
}

void ArmCoG::SetLinkParam(const LinkT armLength, const LinkT offset, const LinkT distortion){
	SetArmLength(armLength);
	SetOffsetParam(offset);
	SetAlphaParam(distortion);
	alpha = distortion;
}

void ArmCoG::SetCoGParam(const double upper, const double forward, const double hand){
	kUpper = upper;
	kForward = forward;
	kHand = hand;
}

void ArmCoG::GetCoGUpper(JointT &curJointRad, CoGT &cog){
	RotMatT bufRot;

	CalcHTM(curJointRad);

	GetHTM(0,1,bufRot);

	RotMatT cogParam = RotMatT::Identity();

	cogParam(1,3) = -sin(alpha[2]) * kUpper;
	cogParam(2,3) = cos(alpha[2]) * kUpper;

	RotMatT buf = bufRot * cogParam;

	std::cout << buf << std::endl;

	for(int i = 0; i < cog.rows(); ++i){
		cog(i,0) = buf(i,3);
	}
}

void ArmCoG::GetCoGForward(JointT &curJointRad, CoGT &cog){
	RotMatT bufRot;

	CalcHTM(curJointRad);

	GetHTM(0,3,bufRot);

	RotMatT cogParam = RotMatT::Identity();

	cogParam(1,3) = -sin(alpha[4]) * kForward;
	cogParam(2,3) = cos(alpha[4]) * kForward;

	RotMatT buf = bufRot * cogParam;

	std::cout << buf << std::endl;

	for(int i = 0; i < cog.rows(); ++i){
		cog(i,0) = buf(i,3);
	}
}

void ArmCoG::GetCoGHand(JointT &curJointRad, CoGT &cog){
	RotMatT bufRot;
	
	CalcHTM(curJointRad);

	GetHTM(0,6,bufRot);

	RotMatT cogParam = RotMatT::Identity();

	cogParam(1,3) = -sin(alpha[7]) * kHand;
	cogParam(2,3) = cos(alpha[7]) * kHand;

	RotMatT buf = bufRot * cogParam;

	std::cout << buf << std::endl;

	for(int i = 0; i < cog.rows(); ++i){
		cog(i,0) = buf(i,3);
	}
}

}	//namespace Trl
