//2017/12/18
//auther:eiki obara

#ifndef __KINE_GRAVITY_COMPENSATION_H__
#define __KINE_GRAVITY_COMPENSATION_H__

#include "kine_centerOfGravity.h"

namespace Trl{

class GravityTorque{
private:
	ArmCoG armcogObj;
	HTM htmObj;

	void GetArmTorque(JointNameT jointName, JointT jointRad, TorqueT &ret);
public:
	GravityTorque(int maxJoint);

	void SetLinkParam(const LinkT &armLength, const LinkT &offset, const LinkT &distortion);
	void SetCoGParam(const CoGT &length, const CoGT &weight);

	void GetJointTorque(const int jointNum, JointT jointRad, TorqueT &ret);

};

GravityTorque::GravityTorque(int maxJoint):armcogObj(maxJoint),htmObj(maxJoint){}

void GravityTorque::SetLinkParam(const LinkT &armLength, const LinkT &offset, const LinkT &distortion){
	armcogObj.SetLinkParam(armLength,offset,distortion);
	htmObj.Set
}
	
void GravityTorque::SetCoGParam(const CoGT &length, const CoGT &weight){
	armcogObj.SetLength(length);
	armcogObj.SetWeight(weight);
}

void GravityTorque::GetArmTorque(JointNameT jointName, JointT jointRad,TorqueT &torq){
	torq=TorqueT::Zero();

	ArmCoG armcogObj(kMaxJoint);

	armcogObj.SetLinkParam(kArmLength,kOffsetLength,kAlphaRad);
	armcogObj.SetLength(kCoGLength);
	armcogObj.SetWeight(kLinkWeight);

	CoGT bufCog = CoGT::Zero();

	switch(jointName){
	case SHOULDER:
		std::cout << "shoulder\n";
		armcogObj.Get(SHOULDER,UPPER,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(0)*kGravityAccel);	//0=upperWeight
		armcogObj.Get(SHOULDER,FORWARD,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(1)*kGravityAccel);	//1=foreWeight
		armcogObj.Get(SHOULDER,HAND,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight

		break;
	case ELBOW:
		std::cout << "elbow\n";
		armcogObj.Get(ELBOW,FORWARD,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(1)*kGravityAccel);	//1=foreWeight
		armcogObj.Get(ELBOW,HAND,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight
	
		break;
	case WRIST:
		std::cout << "wrist\n";
		armcogObj.Get(WRIST,HAND,jointRad,bufCog);
		torq += bufCog.cross(kLinkWeight(2)*kGravityAccel);	//2=handWeight

		break;
	}

	for(int i = 0;i < torq.rows(); ++i){
		if(fabs(torq(i)) < 1.0e-8){
			torq(i) = 0.0;
		}
	}
}

//関節番号、関節角度、返り値のトルク
void GravityTorque::GetJointTorque(const int jointNum,JointT jointRad, TorqueT &ret){
	TorqueT torqBuf;
	
	switch(jointNum){
	case 0:
	case 1:
	case 2:
		GetArmTorque(SHOULDER,jointRad,torqBuf);

		break;
	case 3:
		GetArmTorque(ELBOW,jointRad,torqBuf);
		break;
	case 4:
	case 5:
	case 6:
		GetArmTorque(WRIST,jointRad,torqBuf);
		break;
	default:
		torqBuf = TorqueT::Zero();
		break;
	}




	ret
}

}	//namespace Trl

#endif //__KINE_GRAVITY_COMPENSATION_H__
