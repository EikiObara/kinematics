//2017/09/25
//author : eiki obara
#ifndef __KINE_TRAJECTORY_H__
#define __KINE_TRAJECTORY_H__

#include "kine_cherryCoords.h"
#include "kine_spline.h"
#include "trapeIntrpl.h"
#include "kine_config.h"
#include <iostream>

namespace Trl{

class Trajectory{
private:
	//軌道点
	Eigen::Vector3d points[MAX_SPLINE_POINT];

	//軌道情報格納
	Eigen::Vector3d linearPath;
	Spline splinePath[3];	//xyzの3次元軌道を生成するので決め打ちで3
	
	//変数群
	double controlTime;
	double timeSpan;
	double nowIntpVal;
	double beforeIntpVal;

	//ターゲット格納確認フラグ
	bool startStatus;
	bool endStatus;
	bool viaStatus;
	bool ctrlTimeStatus;
	bool linearStatus;
	bool splineStatus;

public:
	Trajectory();
	~Trajectory();

	void SetStart(Eigen::Vector3d &s);	//始点設定
	void SetEnd(Eigen::Vector3d &e);	//終点設定
	void SetVia(Eigen::Vector3d &v);	//中間点設定

	void SetTimeParam(double _ctrlTime, double _timeSpan);	//制御時間設定

	void GeneratePath(void);

	bool RunLinear(double curTime, Eigen::Vector3d &ret);	//直線|ループで必要回数分回して
	bool RunSpline(double curTime, Eigen::Vector3d &ret);	//曲線|使用してください。
};

Trajectory::Trajectory(){
	for(int i = 0; i < 3; ++i)points[i] = Eigen::Vector3d::Zero();

	startStatus = false;
	endStatus = false;
	viaStatus = false;

	ctrlTimeStatus = false;

	linearStatus = false;
	splineStatus = false;

	controlTime = 0.0;
}

Trajectory::~Trajectory(){}

void Trajectory::SetTimeParam(double _ctrlTime, double _timeSpan){
	controlTime	= _ctrlTime;
	timeSpan	= _timeSpan;

	ctrlTimeStatus = true;
}

void Trajectory::SetStart(Eigen::Vector3d &s){
	points[0] = s;
	startStatus = true;
	//std::cout << " trajectory : start point set->" << startStatus << std::endl;
}

void Trajectory::SetEnd(Eigen::Vector3d &e){
	points[2] = e;
	endStatus = true;
	//std::cout << " trajectory : end point set->" << endStatus << std::endl;
}

void Trajectory::SetVia(Eigen::Vector3d &v){
	points[1] = v;
	viaStatus = true;
	//std::cout << " trajectory : via point set->" << viaStatus << std::endl;
}

void Trajectory::GeneratePath(void){
	if(startStatus == true && endStatus == true){
		if(viaStatus == true){
			Eigen::Matrix<double,MAX_SPLINE_POINT,1> buf[3];
		
			for(int i = 0; i < 3; ++i){
				for(int j = 0; j < MAX_SPLINE_POINT; ++j){
					buf[i](j) = points[j](i);
				}
			}
		
			for(int i = 0; i < 3; ++i){
				splinePath[i].Init(buf[i]);
			}
			splineStatus = true;
		}
	
		linearPath = points[MAX_SPLINE_POINT - 1] - points[0];

		linearStatus = true;
	}else{
		std::cerr << "[ERROR] cannot generate path" << std::endl;
		std::cerr << "[     ] please confirm coords setting" << std::endl;
	}
}

bool Trajectory::RunLinear(double curTime,Eigen::Vector3d &ret){
	if(linearStatus == true){
		for (int i = 0; i < 3; ++i) {
			ret(i)=TrapeInterpolate(linearPath(i),controlTime,timeSpan,curTime);
		}
	}else{
		std::cerr << "[error] trajectory class : runLinear" << std::endl;
		std::cerr << "[     ] non initialized parametor exist" << std::endl;
		ret = Eigen::Vector3d::Zero();

		return false;
	}
	return true;
}

bool Trajectory::RunSpline(double curTime,Eigen::Vector3d &ret){
	if(splineStatus == true){
		if(curTime == 0.0){
			ret = Eigen::Vector3d::Zero();
			nowIntpVal = beforeIntpVal = 0.0;;
		}
		else if(curTime > 0.0 && curTime <= controlTime){
			beforeIntpVal = nowIntpVal;
			nowIntpVal += TrapeInterpolate(ROUTE_LINK,controlTime,timeSpan,curTime);
			
			for(int i = 0; i < 3; ++i){
				ret[i] = splinePath[i].Run(nowIntpVal)	- splinePath[i].Run(beforeIntpVal);
			}
		}
		else if(curTime > controlTime){
			ret = Eigen::Vector3d::Zero();
		}
	}else{
		std::cerr << "[error] trajectory class : runSpline" << std::endl;
		std::cerr << "[     ] non initialized parametor exist " << std::endl;
		ret = Eigen::Vector3d::Zero();

		return false;

	}
	return true;
}

void CalcViaPos(CherryCoords tc, double via2endLength, Eigen::Vector3d &ret) {
	Eigen::Vector3d grasp = Eigen::Vector3d::Zero();
	tc.GetGraspDirection(grasp);
	ret = grasp * via2endLength;
}

}	//namespace Trl
#endif	//__KINE_TRAJECTORY_H__
