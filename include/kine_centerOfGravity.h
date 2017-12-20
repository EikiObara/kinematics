//2017/11/24
//auther : eiki obara

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "kine_htm.h"
#include "kine_defines.h"
#include "kine_robot_param.h"

namespace Trl{

class ArmCoG : protected HTM{
private:
	Eigen::Vector3d cogLength;	//center of gravity length
	Eigen::Vector3d cogWeight;

	LinkT armLength;
	LinkT offset;
	LinkT distortion;

	bool GetCoGParam(JointNameT fromType, CoGNameT toType, Eigen::Vector4d &ret);
public:
	ArmCoG(int maxJoint);

	void SetLinkParam(const LinkT armLength, const LinkT offset, const LinkT distortion);
	void SetLength(const CoGT &length);
	void SetWeight(const CoGT &weight);

	bool Get(JointNameT fromType,CoGNameT toType, JointT &curJointRad, CoGT &cog);
};

ArmCoG::ArmCoG(int maxJoint) : HTM(maxJoint){}

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

bool ArmCoG::Get(JointNameT fromType, CoGNameT toType, JointT &curJointRad, CoGT &cog){
	HtmT bufHtm;

	HTM::CalcHTM(curJointRad);
//	HTM::GetHTMAll();

	//for(int i = 0; i < (int)htm.size(); ++i) std::cout << "htm\n" << htm[i] << std::endl;
	//重心がほしいリンクの一つ手前までのｈｔｍをもらう
	HTM::GetHTM(0,toType,bufHtm);
	Eigen::Vector4d cogParam = Eigen::Vector4d::Zero();
	if(GetCoGParam(fromType,toType,cogParam) == false){
		cog = CoGT::Zero();
		return false;
	}

//	std::cout << "bufHtm\n" << bufRot << std::endl;
//	std::cout << "cogParam\n" << cogParam << std::endl;

	Eigen::Vector4d bufCog = bufHtm * cogParam;

	if(fromType == ELBOW || fromType == WRIST){
		GetHTMAll();
		Eigen::Vector4d jointOffset = Eigen::Vector4d::Zero();
		for(int i = 0; i < 3; ++i) jointOffset(i) = HTM::htm[fromType](i,3);
		bufCog = bufCog - jointOffset;
	}

	//返り値は、要素３のベクトルに直す。外積やらで要素３である必要があるから。
	for(int i = 0, n = 3; i < n; ++i) cog(i,0) = bufCog(i,0);

	//std::cout << "cog\n" << cog << std::endl;

	return true;
}

bool ArmCoG::GetCoGParam(JointNameT fromType,CoGNameT toType, Eigen::Vector4d &ret){
	int typeBuffer = -1;

	if(fromType == SHOULDER){
		if(toType == UPPER){
			typeBuffer = 0;
		}else if(toType == FORWARD){
			typeBuffer = 1;
		}else if(toType == HAND){
			typeBuffer = 2;
		}
	}else if(fromType == ELBOW){
		if(toType == FORWARD){
			typeBuffer = 1;
		}else if(toType == HAND){
			typeBuffer = 2;
		}
	}else if(fromType == WRIST){
		if(toType == HAND){
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
