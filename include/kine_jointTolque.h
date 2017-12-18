//2017/12/18
//auther:eiki obara

#ifndef __KINE_JOINT_TOLQUE_H__
#define __KINE_JOINT_TOLQUE_H__

#include "kine_centerOfGravity.h"
#include "kine_robot_param.h"
#include "kine_defines.h"

namespace Trl{

void GetGravityTolque(JointNameT jointName, JointT curJointRad,TolqueT &tolq){
	tolq=TolqueT::Zero();

	ArmCoG armcog(kMaxJoint);

	armcog.SetLinkParam(kArmLength,kOffsetLength,kAlphaRad);
	armcog.SetLength(kCoGLength);
	armcog.SetWeight(kLinkWeight);

	TolqueT tolqBuf = TolqueT::Zero();
	CoGT bufCog = CoGT::Zero();

	switch(jointName){
	case SHOULDER:
		std::cout << "shoulder\n";
		armcog.Get(SHOULDER,UPPER,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(0)*kGravityAccel);	//0=upperWeight
		armcog.Get(SHOULDER,FORWARD,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(1)*kGravityAccel);	//1=foreWeight
		armcog.Get(SHOULDER,HAND,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight

		break;
	case ELBOW:
		std::cout << "elbow\n";
		armcog.Get(ELBOW,FORWARD,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(1)*kGravityAccel);	//1=foreWeight
		armcog.Get(ELBOW,HAND,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight
	
		break;
	case WRIST:
		std::cout << "wrist\n";
		armcog.Get(WRIST,HAND,curJointRad,bufCog);
		tolqBuf += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight

		break;
	}

	tolq = tolqBuf;
}

}	//namespace Trl

#endif //__KINE_JOINT_TOLQUE_H__
