//2017/11/10
//author : eiki obara

#ifndef __KINE_HAND_POSTURE_H__
#define __KINE_HAND_POSTURE_H__

#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "kine_htm.h"
#include "kine_config.h"
#include "kine_cherryCoords.h"
#include "kine_convertor.h"

namespace Trl{

Eigen::Matrix4d HandRotMat(Eigen::MatrixXd &curJointRad){
	HTM htmObj(curJointRad.rows());
	htmObj.SetArmLength(kALength);
	htmObj.SetAlphaParam(kAlphaRad);
	htmObj.SetOffsetParam(kDLength);
	htmObj.CalcHTM(curJointRad);

	htmObj.GetHTMAll();
	
	return htmObj.htm[htmObj.GetElem()];
}

Eigen::Quaterniond StartPosture(Eigen::MatrixXd &curJointRad){
	Eigen::Matrix4d rotMat = Eigen::Matrix4d::Zero();

	rotMat = HandRotMat(curJointRad);

	Eigen::Quaterniond StartQ = Eigen::Quaterniond::Identity();

	RotMat2Quat(rotMat,StartQ);

	return StartQ;
}

//手先の姿勢にロールピッチヨーで数値を加算して目標姿勢生成
Eigen::Quaterniond EndPosture(Eigen::Vector3d &rpy, Eigen::MatrixXd &curJointRad){
	Eigen::Matrix4d handMat = HandRotMat(curJointRad);
	Eigen::Vector3d originRPY = Eigen::Vector3d::Zero();

	RotMat2RPY(handMat, originRPY);

	Eigen::Matrix4d buf = Eigen::Matrix4d::Zero();

	Eigen::Vector3d addRPY = originRPY + rpy;

	RPY2RotMat(addRPY, buf);

	Eigen::Quaterniond endQ = Eigen::Quaterniond::Identity();

	RotMat2Quat(buf,endQ);

	return endQ;
}

//さくらんぼの3点情報を元にした目標姿勢生成
Eigen::Quaterniond EndPosture(CherryCoords &tc){
	Eigen::Vector3d graspVec = Eigen::Vector3d::Zero();
	Eigen::Vector3d directionX = Eigen::Vector3d::Zero();
	Eigen::Quaterniond endQ = Eigen::Quaterniond::Identity();

	tc.GetGraspDirection(graspVec);

	directionX = tc.top - tc.bottom;

	int checkDirectionX = 0;

	if(checkDirectionX < 3){
		//std::cout << "handMotion class " << std::endl;
		//std::cout << "check direction x : less 3" << std::endl;

		//endQ = Eigen::Quaterniond::FromTwoVectors(directionX,graspVec);
		Eigen::Matrix4d buf;
		DirectVec2RotMat(directionX,graspVec,buf);
		RotMat2Quat(buf,endQ);

	}else if(checkDirectionX >= 3){
		//std::cout << "handMotion class " << std::endl;
		//std::cout << "check direction x : over 3" << std::endl;

		endQ.w() = 0.5;
		endQ.x() = 0.5;
		endQ.y() = 0.5;
		endQ.z() = 0.5;
	}

	return endQ;
}


//肩座標系に対して絶対姿勢を指定して目標姿勢生成
Eigen::Quaterniond EndPosture(Eigen::Matrix4d &rotMat){
//	std::cout << "class : handmotion : end posture absolute" <<std::endl;

	Eigen::Quaterniond endQ = Eigen::Quaterniond::Identity();

	RotMat2Quat(rotMat,endQ);

	return endQ;
}

}	//nmespace Trl

#endif  //__KINE_HAND_POSTURE_H__
