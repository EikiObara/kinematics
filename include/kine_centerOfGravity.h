#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "kine_htm.h"
//#include "kine_coords.h"
#include "kine_config.h"

namespace Trl{

enum FromCoGTypeT{
	shoulder = 0,	//リンクの集約点に依存
	elbow = 2,
	wrist = 5
};

enum ToCoGTypeT{
	upper = 1,	//リンクパラメータに依存
	forward = 3,	//求めたい重心があるリンクがのびる関節を設定
	hand = 6
};

class ArmCoG : private HTM{
private:
	Eigen::Vector3d cogLength;	//center of gravity length
	Eigen::Vector3d cogWeight;

	LinkT armLength;
	LinkT offset;
	LinkT distortion;

	bool GetCoGParam(FromCoGTypeT fromType, ToCoGTypeT toType, Eigen::Vector4d &ret);
public:
	ArmCoG(double maxJoint);

	void SetLinkParam(const LinkT armLength, const LinkT offset, const LinkT distortion);
	void SetLength(const Eigen::Vector3d &length);
	void SetWeight(const Eigen::Vector3d &weight);

	bool Get(FromCoGTypeT fromType,ToCoGTypeT toType, JointT &curJointRad, CoGT &cog);
};

ArmCoG::ArmCoG(double maxJoint) : HTM(maxJoint){}

void ArmCoG::SetLinkParam(const LinkT armLength, const LinkT offset, const LinkT distortion){
	SetArmLength(armLength);
	SetOffsetParam(offset);
	SetAlphaParam(distortion);

	this->armLength = armLength;
	this->offset = offset;
	this->distortion = distortion;
}

void ArmCoG::SetLength(const Eigen::Vector3d &length){
	cogLength = length;
}

void ArmCoG::SetWeight(const Eigen::Vector3d &weight){
	cogWeight = weight;
}

bool ArmCoG::Get(FromCoGTypeT fromType, ToCoGTypeT toType, JointT &curJointRad, CoGT &cog){
	RotMatT bufRot;

	HTM::CalcHTM(curJointRad);

	HTM::GetHTMAll();

	//for(int i = 0; i < (int)htm.size(); ++i) std::cout << "htm\n" << htm[i] << std::endl;

	//重心がほしいリンクの一つ手前までのｈｔｍをもらう
	HTM::GetHTM(0,toType,bufRot);

	Eigen::Vector4d cogParam = Eigen::Vector4d::Zero();

	if(GetCoGParam(fromType,toType,cogParam) == false){
		cog = CoGT::Zero();
		return false;
	}

	std::cout << "bufRot\n" << bufRot << std::endl;

	std::cout << "cogParam\n" << cogParam << std::endl;

	Eigen::Vector4d bufCog = bufRot * cogParam;

	std::cout << "cog\n" << bufCog << std::endl;

	if(fromType == elbow || fromType == wrist){
		GetHTMAll();
		Eigen::Vector4d jointOffset = Eigen::Vector4d::Zero();
		for(int i = 0; i < 3; ++i) jointOffset(i) = HTM::htm[fromType](i,3);

		std::cout << "elbow\n" << jointOffset << std::endl;

		bufCog = bufCog - jointOffset;
	}

	for(int i = 0, n = 3; i < n; ++i) cog(i,0) = bufCog(i,0);


	return true;
}

bool ArmCoG::GetCoGParam(FromCoGTypeT fromType,ToCoGTypeT toType, Eigen::Vector4d &ret){
	int typeBuffer = -1;

	if(fromType == shoulder){
		if(toType == upper){
			typeBuffer = 0;
		}else if(toType == forward){
			typeBuffer = 1;
		}else if(toType == hand){
			typeBuffer = 2;
		}
	}else if(fromType == elbow){
		if(toType == forward){
			typeBuffer = 1;
		}else if(toType == hand){
			typeBuffer = 2;
		}
	}else if(fromType == wrist){
		if(toType == hand){
			typeBuffer = 2;
		}
	}

	if(typeBuffer == -1){
		std::cout << "[ERROR] ArmCoG : GetParam : Wrong parameter " << std::endl;
		ret = Eigen::Vector4d::Zero();

		return false;
	}

	Eigen::Vector4d lengthParam = Eigen::Vector4d::Zero();
	
	lengthParam(0) = 0.0;
	lengthParam(1) = -cogLength[typeBuffer] * sin(distortion[toType+1]);
	lengthParam(2) = cogLength[typeBuffer] * cos(distortion[toType+1]);
	lengthParam(3) = 1.0;

	ret = lengthParam;

	return true;
}

}	//namespace Trl
