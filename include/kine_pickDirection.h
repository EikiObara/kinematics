
#ifndef _KINE_PICK_DIRECTION_H_
#define _KINE_PICK_DIRECTION_H_

namespace Trl{

enum Direction{
	PICK_BACK = 0,
	PICK_RIGHT,
	PICK_LEFT
};

const double DIRECTION_THRESHOLD = 0.01;
const double LENGTH = 100;

void PickDirection(Eigen::Vector3d &topPoint,Eigen::Vector3d &btmPoint,Eigen::Vector3d &returnV){
	Eigen::Vector3d buf = Eigen::Vector3d::Zero();

	buf = topPoint - btmPoint;

	buf.normalize();

	//+なら右に動かす
	if(buf(2) > DIRECTION_THRESHOLD){
		returnV(0) = 0;
		returnV(1) = 0;
		returnV(2) = 80;

	//-なら左に動かす
	}else if(buf[2] < DIRECTION_THRESHOLD){
		returnV(0) = 0;
		returnV(1) = 0;
		returnV(2) = -80;
	}else{
		returnV(0) = -80;
		returnV(1) = 0;
		returnV(2) = 0;
	}
}

void PickDirectionManual(Direction type,Eigen::Vector3d &returnV){
	switch(type){
	case PICK_RIGHT:
		returnV(0) = 0;
		returnV(1) = 0;
		returnV(2) = LENGTH;
		break;
	case PICK_LEFT:
		returnV(0) = 0;
		returnV(1) = 0;
		returnV(2) = -1 * LENGTH;
		break;
	case PICK_BACK:
		returnV(0) = -1 * LENGTH;
		returnV(1) = 0;
		returnV(2) = 0;
		break;
	default:
		returnV(0) = 0;
		returnV(1) = 0;
		returnV(2) = 0;
		break;
	}
}

}	//namespace Trl

#endif //_KINE_PICK_DIRECTION_H_
