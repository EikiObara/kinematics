#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "kine_coords.h"
#include "kine_trajectory.h"
#include "kine_handVelocity.h"
#include "kine_handPosture.h"

typedef Eigen::MatrixXd JointT;
typedef Eigen::Vector3d CoordsT;

typedef Eigen::Vector3d EulerT;

typedef Eigen::Quaterniond PostureT;

typedef Eigen::Matrix<double,Eigen::Dynamic,6> HandPathT;

namespace Trl{

class VelGenerator{
private:
	int dof;	//degree of freedom
public:
	VelGenerator(int dof) : dof(dof){}

	//tarC = target coords, tarP = target posture
	bool Straight(JointT curJ,CoordsT tarC,PostureT tarP,
		const double timeSpan,const double ctrlTime,HandPathT &hv);
	bool Spline(JointT curJ,CoordsT viaC,CoordsT tarC,PostureT tarP,
		const double ctrlTime,const double timeSpan,HandPathT &hv);
};

bool VelGenerator::Straight(JointT curJ,CoordsT tarC,PostureT tarP,
	const double ctrlTime,const double timeSpan,HandPathT &retHP){

	if(std::isinf(ctrlTime/timeSpan)){
		std::cerr << "[ERROR] VelocityGenerate : Straight" << std::endl;
		std::cerr << "[     ] timeSpan is an inappropriate value."<< std::endl;

		return false;
	}
	
	//Eigenの二次元配列の初期化
	retHP.resize((int)((ctrlTime/timeSpan)+2),6);
	//retHP = Eigen::Matrix<double,(int)((ctrlTime/timeSpan)+2),6>::Zero();
	
	//指先位置軌道の生成
	Eigen::Vector3d finger;
	Trl::Coords crd(dof);
	crd.InitCoords(curJ);
	crd.GetFinger(finger);

	Trl::Trajectory tr;
	tr.SetTimeParam(ctrlTime,timeSpan);
	tr.SetStart(finger);
	tr.SetEnd(tarC);
	tr.GeneratePath();

	//手先姿勢軌道の生成
	Eigen::Quaterniond st = StartPosture(curJ);

	Trl::HandVelocity hv;
	hv.SetTimeParam(ctrlTime,timeSpan);
	hv.SetStart(st);
	hv.SetEnd(tarP);

	//返り値の入れ物
	Eigen::Vector3d retVelPstn = Eigen::Vector3d::Zero();
	Eigen::Vector3d retVelPstr = Eigen::Vector3d::Zero();

	int counter = 0;

	for(double curTime = 0; curTime <= ctrlTime; curTime += timeSpan){
		tr.RunLinear(curTime,retVelPstn);
		hv.Run(curTime,retVelPstr);

		Eigen::Matrix<double,6,1> buf;
		for(int i = 0; i < 3; ++i) buf(i,0) = retVelPstn(i,0);
		for(int i = 0; i < 3; ++i) {
			double iAdd = i + 3;
			buf(iAdd,0) = retVelPstr(i,0);
		}

		for(int i = 0; i < buf.rows(); ++i) retHP(counter,i) = buf(i,0);
		++counter;
	}
	
	return true;
}

bool VelGenerator::Spline(JointT curJ,CoordsT viaC,CoordsT tarC,PostureT tarP,
	const double ctrlTime,const double timeSpan,HandPathT &retHP){

	if(std::isinf(ctrlTime/timeSpan)){
		std::cerr << "[ERROR] VelocityGenerate : Spline" << std::endl;
		std::cerr << "[     ] timeSpan is an inappropriate value."<< std::endl;

		return false;
	}

	retHP.resize((int)((ctrlTime/timeSpan)+2),6);
	//retHP = Eigen::Matrix<double,(int)((ctrlTime/timeSpan)+2),6>::Zero();

	//指先位置軌道の生成
	Eigen::Vector3d finger;
	Trl::Coords crd(dof);
	crd.InitCoords(curJ);
	crd.GetFinger(finger);

	Trl::Trajectory tr;
	tr.SetTimeParam(ctrlTime,timeSpan);
	tr.SetStart(finger);
	tr.SetVia(viaC);
	tr.SetEnd(tarC);
	tr.GeneratePath();

	//手先姿勢軌道の生成
	Eigen::Quaterniond st = StartPosture(curJ);

	Trl::HandVelocity hv;
	hv.SetTimeParam(ctrlTime,timeSpan);
	hv.SetStart(st);
	hv.SetEnd(tarP);

	//返り値の入れ物
	Eigen::Vector3d retVelPstn = Eigen::Vector3d::Zero();
	Eigen::Vector3d retVelPstr = Eigen::Vector3d::Zero();

	int counter = 0;

	for(double curTime = 0; curTime <= ctrlTime; curTime += timeSpan){
		tr.RunLinear(curTime,retVelPstn);
		hv.Run(curTime,retVelPstr);

		Eigen::Matrix<double,6,1> buf;
		for(int i = 0; i < 3; ++i) buf(i,0) = retVelPstn(i,0);
		for(int i = 0; i < 3; ++i) {
			double iAdd = i + 3;
			buf(iAdd,0) = retVelPstr(i,0);
		}

		for(int i = 0; i < buf.rows(); ++i) retHP(counter,i) = buf(i,0);
		++counter;
	}

	return true;
}

}	//namespace Trl
