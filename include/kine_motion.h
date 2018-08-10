//2017/10/26
// author : eiki obara

#ifndef __KINE_MOTION_H__
#define __KINE_MOTION_H__

#include "kine_htm.h"

namespace Trl{

class Motion{
protected:
	HTM htmObj;

	virtual void CalcJacob(void) = 0;
	virtual bool CalcIJacob(void) = 0;
public:
	Motion(int jointNum,Eigen::MatrixXd aLength,Eigen::MatrixXd dLegnth,Eigen::MatrixXd alpha);
	virtual bool Run(Eigen::MatrixXd &curJointRad,Eigen::MatrixXd &handVel, Eigen::MatrixXd &ret) = 0;
};

Motion::Motion(int jointNum,Eigen::MatrixXd aLength,Eigen::MatrixXd dLength,Eigen::MatrixXd alpha):htmObj(jointNum){
	htmObj.SetArmLength(aLength);
	htmObj.SetOffsetParam(dLength);
	htmObj.SetAlphaParam(alpha);
}

}	//namespace Trl

#endif //__KINE_MOTION_H__
