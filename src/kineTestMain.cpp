#include <iostream>
#include <string>
#include <algorithm>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "loggerTrl.h"

//検定中
#include "kine_basicMotion.h"
#include "kine_coords.h"
#include "kine_config.h"
#include "torque.h"
#include "translator.h"
#include "myAlgorithm.h"

const double gravityAcceleration = 9.80665;

Eigen::MatrixXd SetCurJointRad(){
	Eigen::MatrixXd curJointRad(Trl::kMaxJoint,1);
	curJointRad(0,0) = DegToRad(0);				//DegToRad(***here***)←変更する（degree値）
	curJointRad(1,0) = DegToRad(-90);			//pose 1
	curJointRad(2,0) = DegToRad(-36.87);		//
	curJointRad(3,0) = DegToRad(0);				//
	curJointRad(4,0) = DegToRad(36.87);			//
	curJointRad(5,0) = DegToRad(0);				//

//	curJointRad(0,0) = DegToRad(0);				//
//	curJointRad(1,0) = DegToRad(-66.4218);		//pose 2
//	curJointRad(2,0) = DegToRad(44.15635);		//
//	curJointRad(3,0) = DegToRad(0);				//
//	curJointRad(4,0) = DegToRad(-66.4218);		//
//	curJointRad(5,0) = DegToRad(0);				//

//	curJointRad(0,0) = DegToRad(0);				//
//	curJointRad(1,0) = DegToRad(-100.55);		//pose 3
//	curJointRad(2,0) = DegToRad(90-68.8998);		//
//	curJointRad(3,0) = DegToRad(0);				//
//	curJointRad(4,0) = DegToRad(-10.55);		//
//	curJointRad(5,0) = DegToRad(0);				//

	return curJointRad;
}

Eigen::MatrixXd SetMassForHand(){
	Eigen::MatrixXd handForce(6,1);
	handForce(0,0) = 0.0;		//x方向の力
	handForce(1,0) = 0.0;			//y方向の力
	handForce(2,0) = 30.45 - 2.0 * gravityAcceleration;	//z方向の力(上が正)
	handForce(3,0) = 0.0;							
	handForce(4,0) = 0.0;							
	handForce(5,0) = 0.0;
	return handForce;
}

Eigen::MatrixXd SetTension(){
	Eigen::MatrixXd vec(7,1);
	vec(0,0) = 0.0;			//torque1
	vec(1,0) = 0.0;			//T1
	vec(2,0) = 0.0;			//T2
	vec(3,0) = 0.0;			//T3
	vec(4,0) = 0.0;			//T4
	vec(5,0) = 0.0;			//T5
	vec(6,0) = 0.0;			//T6
	return vec;
}

Eigen::MatrixXd SetNullSpace(){
	Eigen::MatrixXd vec(7,1);
	vec(0,0) = 0.0;
	vec(1,0) = 8.0;
	vec(2,0) = 4.0;
	vec(3,0) = 1.0;
	vec(4,0) = 1.0;
	vec(5,0) = 1.0;
	vec(6,0) = 1.0;
	return vec;
}

int main(){
	Eigen::MatrixXd jr = SetCurJointRad();
	Eigen::MatrixXd ret(Trl::kMaxJoint,1);

	Trl::BasicMotion bm(Trl::kMaxJoint,Trl::kALength,Trl::kDLength,Trl::kAlphaRad);
	Trl::Coords coord(Trl::kMaxJoint,Trl::kALength,Trl::kDLength, Trl::kAlphaRad);

	std::cout << "elbow Position\n" << coord.GetElbow(jr) << std::endl;
	std::cout << "wrist position\n" << coord.GetWrist(jr) << std::endl;
	std::cout << "finger position\n" << coord.GetFinger(jr) << std::endl;

	Eigen::MatrixXd jacob = bm.GetJacobian(jr);
	Eigen::MatrixXd iJacob = bm.GetInverseJacobian(jr);
	Eigen::MatrixXd force = SetMassForHand();
	Eigen::MatrixXd pulleyMat = GetPulleyMatrix();
	Eigen::MatrixXd invPulleyMat = GetPseudoMatrix(pulleyMat);
	Eigen::MatrixXd nullVec = SetNullSpace();
	Eigen::MatrixXd originTorque = invPulleyMat * jacob.transpose() * force;
	unsigned int minimum = FindMinimum(originTorque, nullVec);
	Eigen::MatrixXd interpolateVec = -nullVec * (originTorque(minimum,0) / nullVec(minimum,0)) * 1.0;
	Eigen::MatrixXd adjustedTorque = originTorque + interpolateVec;
	Eigen::MatrixXd handForceRef = jacob.transpose().inverse() * pulleyMat * (originTorque + interpolateVec);
 


	std::cout << "jacobian\n" << jacob << std::endl;
	std::cout << "inverse jacobian\n" << iJacob << std::endl;

	std::cout << "pulley Matrix\n" << pulleyMat << std::endl;
	std::cout << "pseudo inverse pulley Matrix\n" << invPulleyMat<< std::endl;
//	std::cout << "null space" << nullSpace << std::endl;
//	std::cout << "null space vector" << nullSpace * vec << std::endl;

	std::cout << "origin tension\n" << originTorque << std::endl;

	std::cout << "adjusted tension\n" << adjustedTorque << std::endl;

	std::cout << "origin torque\n" << pulleyMat * (originTorque) << std::endl;

	std::cout << "force\n" << handForceRef << std::endl;

	
		return 0;
}

