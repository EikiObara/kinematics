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

	virtual void CalcJacob(void);
	virtual bool CalcIJacob(void);
public:
	BasicMotion(int maxJoint);
	virtual bool Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret);
	
};

BasicMotion::BasicMotion(int maxJoint) : Motion(maxJoint){}

void BasicMotion::CalcJacob(void){
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

	Eigen::Matrix<double,6,6> diagonal = Eigen::Matrix<double,6,6>::Zero();

	diagonal = jacob * jacob.transpose();

	if(fabs(diagonal.determinant()) < kCompareZero)return false;

	Eigen::Matrix<double,6,6> iden = Eigen::Matrix<double,6,6>::Identity();
	Eigen::Matrix<double,6,6> inverseDiago = Eigen::Matrix<double,6,6>::Zero();

	inverseDiago = diagonal.partialPivLu().solve(iden);

	iJacob = jacob.transpose() * inverseDiago;

	return true;
}

bool BasicMotion::Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret){
	jacob	= Eigen::MatrixXd(6,7);
	iJacob	= Eigen::MatrixXd(7,6);

	htmObj.CalcHTM(curJointRad);
	htmObj.GetHTMAll();

	CalcJacob();

	//std::cout << "jacob" << std::endl;
	//std::cout << jacob << std::endl;

	if(CalcIJacob() == false){
		return false;
	}

	Eigen::Matrix<double,6,1> buf = Eigen::Matrix<double,6,1>::Zero();

	for(int i = 0; i < 6; ++i) buf(i) = handVel(i);

	ret = iJacob * buf;

	return true;
}

}	//namespace Trl

#endif //__KINE_BASICMOTION_H__
