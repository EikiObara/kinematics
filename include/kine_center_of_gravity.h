//2017/11/24
//auther : eiki obara

#ifndef __KINE_CENTER_OF_GRAVITY_H__
#define __KINE_CENTER_OF_GRAVITY_H__

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "kine_htm.h"
#include "kine_defines.h"

namespace Trl{

class ArmCoG{
private:
	Eigen::Vector3d cogLength;	//center of gravity length
	Eigen::Vector3d cogWeight;

	bool GetCoGVecParam(JointNameT fromType,CoGNameT toType,HTM htmObj,Eigen::Vector4d &ret);
public:
	void SetCoGLength(CoGT length);

	bool Get(JointNameT fromType,CoGNameT toType,HTM htmObj,CoGT &cog);
};

void ArmCoG::SetCoGLength(CoGT length){
	cogLength = length;
}

bool ArmCoG::Get(JointNameT fromType,CoGNameT toType,HTM htmObj,CoGT &cog){
	//肩原点から重心がほしいリンクの一つ手前までのｈｔｍをもらう
	HtmT bufHtm = HtmT::Zero();
	htmObj.GetHTM(0,toType,bufHtm);

	//重心のパラメータをもつベクトルを生成する
	Eigen::Vector4d cogParam = Eigen::Vector4d::Zero();

	if(GetCoGVecParam(fromType,toType,htmObj,cogParam) == false){
		cog = CoGT::Zero();
		return false;
	}

	//std::cout << "bufHtm\n" << bufHtm << std::endl;
	//std::cout << "cogParam\n" << cogParam << std::endl;

	//肩原点からの重心位置ベクトルを生成
	Eigen::Vector4d bufCog = bufHtm * cogParam;

	//肘、手首を始点にした場合
	if(fromType == ELBOW || fromType == WRIST){
		htmObj.GetHTMAll();
		Eigen::Vector4d jointOffset = Eigen::Vector4d::Zero();
		for(int i = 0; i < 3; ++i) jointOffset(i) = htmObj.htm[fromType](i,3);
		bufCog = bufCog - jointOffset;
	}

	//std::cout << "bufCog\n" << bufCog << std::endl;

	//返り値は、要素３のベクトルに直す。外積やらで要素３である必要があるから。
	for(int i = 0, n = 3; i < n; ++i) cog(i,0) = bufCog(i,0);

	return true;
}

//リンクによって変わる重心のパラメータを算出
bool ArmCoG::GetCoGVecParam(JointNameT fromType,CoGNameT toType,HTM htmObj,Eigen::Vector4d &ret){
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

	//std::cout << htmObj.distortion(toType+1) << std::endl;
	//std::cout << " type buffer\n " << typeBuffer << std::endl;
	
	lengthParam(0) = 0.0;
	lengthParam(1) = -cogLength(typeBuffer,0) * sin(htmObj.distortion(toType+1));
	lengthParam(2) = cogLength(typeBuffer,0) * cos(htmObj.distortion(toType+1));
	lengthParam(3) = 1.0;

	ret = lengthParam;

	return true;
}

}	//namespace Trl

#endif //__KINE_CENTER_OF_GRAVITY_H__
