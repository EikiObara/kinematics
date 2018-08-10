// 2017/09/25
// author: eiki obara
#ifndef __KINE_BASICMOTION_H__
#define __KINE_BASICMOTION_H__

#include "kine_motion.h"

namespace Trl{

class BasicMotion : public Motion {
private:
	Eigen::MatrixXd jacob;
	Eigen::MatrixXd iJacob;

	const int maxJoint;
	int jacobSize;

	virtual void CalcJacob();
	virtual bool CalcIJacob();
public:
	BasicMotion(int maxJoint,Eigen::MatrixXd aLength, Eigen::MatrixXd dLength, Eigen::MatrixXd alpha);
	Eigen::MatrixXd GetJacobian(Eigen::MatrixXd &jointRad);
	Eigen::MatrixXd GetInverseJacobian(Eigen::MatrixXd &jointRad);
	virtual bool Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret);
};

BasicMotion::BasicMotion(int maxJoint,Eigen::MatrixXd aLength, Eigen::MatrixXd dLength, Eigen::MatrixXd alpha) :
	Motion(maxJoint, aLength, dLength, alpha),
	maxJoint(maxJoint),
	jacobSize(maxJoint),
	jacob(Eigen::MatrixXd(maxJoint,maxJoint)),
	iJacob(Eigen::MatrixXd(maxJoint,maxJoint))
{
	if(maxJoint > 6){
		jacob.resize(6,maxJoint);
		iJacob.resize(maxJoint,6);
		jacobSize = 6;
	}
}

void BasicMotion::CalcJacob(){
	for(int joint = 0; joint < htmObj.GetElem(); ++joint){
		Eigen::Vector3d posVec = Eigen::Vector3d::Zero();

		for(int i = 0; i < 3; ++i){
			posVec(i) = htmObj.htm[htmObj.GetElem()](i,3) - htmObj.htm[joint](i,3);
		}

		Eigen::Vector3d zAxis = Eigen::Vector3d::Zero();

		for(int i = 0; i < 3; ++i){
			zAxis(i) = htmObj.htm[joint](i,2);
		}

		Eigen::Vector3d crossBuf = Eigen::Vector3d::Zero();
		crossBuf = zAxis.cross(posVec);

		for(int i = 0; i < 3; ++i){
			jacob(i,joint) = crossBuf(i);
			jacob(i+3,joint) = htmObj.htm[joint](i,2);
		}
	}

	for(int i = 0; i < jacob.rows(); ++i){
		for(int j = 0; j < jacob.cols(); ++j){
			if(fabs(jacob(i,j)) < kCompareZero){
				jacob(i,j) = 0.0;
			}
		}
	}
}

bool BasicMotion::CalcIJacob(void){
	if(htmObj.GetElem() < 6){
		std::cout << "* class : basic motion : calc IJacob *" << std::endl;
		std::cout << "* homogeneous translate matrix elements *" << std::endl;
		std::cout << "* invalid *" << std::endl;
		return false;
	}

	Eigen::MatrixXd diagonal(jacobSize,jacobSize);
	diagonal = jacob * jacob.transpose();

	if(fabs(diagonal.determinant()) < kCompareZero)	return false;

	Eigen::MatrixXd iden(jacobSize,jacobSize);
	Eigen::MatrixXd inverseDiago(jacobSize,jacobSize);

	inverseDiago = diagonal.partialPivLu().solve(iden);

	iJacob = jacob.transpose() * inverseDiago;

	return true;
}

bool BasicMotion::Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret){
	htmObj.CalcHTM(curJointRad);
	htmObj.DispOM();

	htmObj.GetHTMAll();
	htmObj.DispHTM();

	CalcJacob();

	//std::cout << "jacob" << std::endl;
	//std::cout << jacob << std::endl;

	if(CalcIJacob() == false)	return false;

	ret = iJacob * handVel;

	return true;
}

Eigen::MatrixXd BasicMotion::GetJacobian(Eigen::MatrixXd &jointRad){
	htmObj.CalcHTM(jointRad);
	htmObj.GetHTMAll();
	//htmObj.DispOM();
	//htmObj.DispHTM();
	CalcJacob();
	return jacob;
}

Eigen::MatrixXd BasicMotion::GetInverseJacobian(Eigen::MatrixXd &jointRad){
	htmObj.CalcHTM(jointRad);
	htmObj.GetHTMAll();
	CalcJacob();
	if(!CalcIJacob())	std::cout << "cannot create inverse matrix" << std::endl;
	return iJacob;
}

}	//namespace Trl

#endif //__KINE_BASICMOTION_H__
