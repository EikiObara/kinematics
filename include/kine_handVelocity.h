#ifndef __KINE__HAND_VELOCITY_H__
#define __KINE__HAND_VELOCITY_H__
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "trapeIntrpl.h"
#include "kine_convertor.h"


namespace Trl{

bool QuatJudge(Eigen::Quaterniond &q){
	int error = 0;
	if(fabs(q.w()) < kCompareZero) ++error;
	if(fabs(q.x()) < kCompareZero) ++error;
	if(fabs(q.y()) < kCompareZero) ++error;
	if(fabs(q.z()) < kCompareZero) ++error;

	if(error == 4){
		return false;
	}else{
		return true;
	}
}

Eigen::Quaterniond QuatSub(Eigen::Quaterniond &q1, Eigen::Quaterniond &q2){
	Eigen::Quaterniond ret;

	ret.w() = q1.w() - q2.w();
	ret.x() = q1.x() - q2.x();
	ret.y() = q1.y() - q2.y();
	ret.z() = q1.z() - q2.z();

	return ret;
}

void QuatDisp(Eigen::Quaterniond &q){
	std::cout << q.w() << std::endl;
	std::cout << q.x() << std::endl;
	std::cout << q.y() << std::endl;
	std::cout << q.z() << std::endl;
}


class HandVelocity{
private:
	Eigen::Quaterniond startQ;
	Eigen::Quaterniond endQ;

	Eigen::Quaterniond nowQ;
	Eigen::Quaterniond beforeQ;

	bool startQPerm;	//start quaternion permission
	bool endQPerm;		//end quaternion permission
	bool timePerm;
	bool runPerm;	//trueなら計算する

	double ctrlTime;
	double timeSpan;
	double trapeVal;


public:
	HandVelocity();

	void SetTimeParam(double ctrlTime, double timeSpan);

	void SetStart(Eigen::Quaterniond &start);
	void SetEnd(Eigen::Quaterniond &end);

	bool Run(double curTime, Eigen::Vector3d &ret);
};

HandVelocity::HandVelocity(){
	startQPerm = false;
	endQPerm = false;
	runPerm = false;
	timePerm = false;

	trapeVal = 0.0;

	nowQ = Eigen::Quaterniond::Identity();
	beforeQ = Eigen::Quaterniond::Identity();

}

void HandVelocity::SetTimeParam(double _ctrlTime, double _timeSpan){
	ctrlTime = _ctrlTime;
	timeSpan = _timeSpan;

	timePerm = true;
}

void HandVelocity::SetStart(Eigen::Quaterniond &_start){
	if(QuatJudge(_start) == false){
		return;

	}

	startQ = _start;
	startQPerm = true;
}

void HandVelocity::SetEnd(Eigen::Quaterniond &_end){
	if(QuatJudge(_end) == false){
		return;
	}

	endQ = _end;
	endQPerm = false;
}

bool HandVelocity::Run(double curTime,Eigen::Vector3d &ret){

	if(curTime == 0){
		//現在および目標姿勢の初期化判定
		if(startQPerm == false && endQPerm == false){
			std::cerr << "[ERROR] HandVelocity : Run" << std::endl;
			std::cerr << "[     ] Posture Parameter is NOT set" << std::endl;
			std::cerr << "[     ] Please confirm parameter setting" << std::endl;
	
			return false;
		}

		if(timePerm == false){
			std::cerr << "[ERROR] HandVelocity : Run" << std::endl;
			std::cerr << "[     ] Time Parameter is NOT set" << std::endl;
			std::cerr << "[     ] Please confirm parameter setting" << std::endl;

			return false;
		}
		
		//同じ姿勢か判定(ここでrunPermがtrueにならないと姿勢計算は行われない)
		Eigen::Quaterniond buf = QuatSub(startQ,endQ);
		if(QuatJudge(buf) == true) runPerm = true;
	}

	//計算部
	if(runPerm == true){
		if(curTime <= timeSpan){
			beforeQ = startQ;
			double buf = TrapeInterpolate(1,ctrlTime,timeSpan,timeSpan);
			nowQ = startQ.slerp(buf,endQ);
			ret = Eigen::Vector3d::Zero();
		}else if(curTime > timeSpan){
			Eigen::Vector3d nowEuler = Eigen::Vector3d::Zero();
			Eigen::Vector3d beforeEuler = Eigen::Vector3d::Zero();
	
			beforeQ = nowQ;
	
			trapeVal += TrapeInterpolate(1,ctrlTime,timeSpan,curTime);

			//std::cout << trapeVal << std::endl;
	
			nowQ = startQ.slerp(trapeVal,endQ);
	
			Quat2Euler(nowQ,nowEuler);
			Quat2Euler(beforeQ,beforeEuler);

			//std::cout <<"n\n"<<  nowEuler << std::endl;
			//std::cout <<"b\n"<< beforeEuler << std::endl;
	
			Eigen::Vector3d eulerVel = Eigen::Vector3d::Zero();
			eulerVel = nowEuler - beforeEuler;
			//std::cout << eulerVel << std::endl;
	
			Eigen::Vector3d buf = Eigen::Vector3d::Zero();

			Euler2Angular(nowEuler,eulerVel,buf);
			
			//std::cout << "debag : roll pitch yaw" << std::endl;
			for(int i = 0; i < 3; ++i) { ret(i) = -buf(i);}
		}
		else if(curTime > ctrlTime) {ret = Eigen::Vector3d::Zero();}
	}

	return true;
}

}	//namespace Trl

#endif	//	__KINE_HAND_VELOCITY_H__
