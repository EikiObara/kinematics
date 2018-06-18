//2017/09/25
//author : eiki obara

#ifndef __KINE_SELFMOTION_H__
#define __KINE_SELFMOTION_H__

#include "kine_motion.h"

namespace Trl{

class SelfMotion : private Motion{
private:
	Eigen::MatrixXd jacob;
	Eigen::MatrixXd iJacob;

	void CalcJacob(void);
	bool CalcIJacob(void);
public:
	SelfMotion(int maxJoint);
	bool Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret);

};

SelfMotion::SelfMotion(int maxJoint) : Motion(maxJoint){}

void SelfMotion::CalcJacob(void){

	for(int joint = 0; joint < htmObj.GetElem(); ++joint){
		Eigen::Vector3d posVec = Eigen::Vector3d::Zero();
		for(int i = 0; i < 3; ++i){
			posVec(i) = htmObj.htm[htmObj.GetElem()](i,3) - htmObj.htm[joint](i,3);
		}
		Eigen::Vector3d zAxis = Eigen::Vector3d::Zero();

		for(int i = 0; i < 3; ++i) zAxis(i) = htmObj.htm[joint](i,2);

		Eigen::Vector3d crossBuf = Eigen::Vector3d::Zero();

		crossBuf = zAxis.cross(posVec);

		for(int i = 0; i < 3; ++i){
			jacob(i,joint) = crossBuf(i);
			jacob(i+3,joint) = htmObj.htm[joint](i,2);
		}
	}

	for(size_t i = 0,n = jacob.cols(); i < n; ++i){ jacob(6,i) = 0.0;}
		
	jacob(6,2) = 1.0;

	for(int i = 0; i < htmObj.GetElem(); ++i){
		for(int j = 0; j < htmObj.GetElem(); ++j){
			if(fabs(jacob(i,j)) < kCompareZero){
				jacob(i,j) = 0.0;
			}
		}
	}
}

bool SelfMotion::CalcIJacob(void){
	Eigen::MatrixXd iden = Eigen::MatrixXd(htmObj.GetElem(),htmObj.GetElem());
	iden = Eigen::MatrixXd::Identity(htmObj.GetElem(),htmObj.GetElem());

//	Eigen::MatrixXd buf = jacob * jacob.transpose();
//	Eigen::MatrixXd inverseDiago = buf.partialPivLu().solve(iden);
//	iJacob = jacob.transpose() * inverseDiago;

	if(fabs(jacob.determinant()) < kCompareZero){
		std::cout << "Kinematics::inverse Jacobian" << std::endl;
		std::cout << "determinant cannot calculate" << std::endl;
		return false;
	}

	iJacob = jacob.inverse();

	for(int i = 0; i < iJacob.rows(); ++i){
		for(int j = 0; j < iJacob.cols(); ++j){
			if(fabs(iJacob(i,j)) < kCompareZero){
				iJacob(i,j) = 0.0;
			}
		}
	}

	return true;
}

bool SelfMotion::Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel,Eigen::MatrixXd &ret){
	htmObj.CalcHTM(curJointRad);
	htmObj.GetHTMAll();

	jacob  = Eigen::MatrixXd(7,7);
	iJacob = Eigen::MatrixXd(7,7);

	CalcJacob();

	ret = Eigen::MatrixXd(handVel.rows(),1);

	//std::cout << "jacob\n" << jacob << std::endl;

	if(CalcIJacob() == false) return false;

//	std::cout << "handVel\n" << handVel << std::endl;
//	std::cout << "iJacob\n" << iJacob << std::endl;

//	for(int i = 0; i < iJacob.rows(); ++i){
//		for(int j = 0; j < handVel.rows(); ++j){
//			ret(i,0) += iJacob(i,j) * handVel(j,0);
//		}
//	}

	ret = iJacob * handVel;

	return true;
}

}	//namespace Trl


#endif //__KINE_SELFMOTION_H__
