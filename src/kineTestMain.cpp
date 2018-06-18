#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "loggerTrl.h"

//#include "kinematics.h"

//合格
#include "kine_coords.h"
#include "trapeIntrpl.h"
#include "kine_cherryCoords.h"
#include "kine_handPosture.h"
#include "kine_handVelocity.h"
#include "kine_htm.h"

//検定中
#include "kine_velocityGenerator.h"

#include "kine_basicMotion.h"
#include "kine_selfMotion.h"
#include "kine_config.h"


void SetCurJointRad(Eigen::MatrixXd &curJointRad){
	curJointRad(0,0) = 50 * M_PI / 180 - M_PI / 2;
	curJointRad(1,0) = 0 * M_PI / 180 + M_PI / 2;
	curJointRad(2,0) = 0 * M_PI / 180 + M_PI / 2;
	curJointRad(3,0) = 90 * M_PI / 180;
	curJointRad(4,0) = 0 * M_PI / 180;
	curJointRad(5,0) = -50 * M_PI / 180;
	curJointRad(6,0) = 0 * M_PI / 180;
}

void SetCurJointRad(std::vector<double> &curJointRad){
	curJointRad[0] = 50 * M_PI / 180 - M_PI / 2;
	curJointRad[1] = 0 * M_PI / 180 + M_PI / 2;
	curJointRad[2] = 0 * M_PI / 180 + M_PI / 2;
	curJointRad[3] = 90 * M_PI / 180;
	curJointRad[4] = 0 * M_PI / 180;
	curJointRad[5] = -50 * M_PI / 180;
	curJointRad[6] = 0 * M_PI / 180;
}

void SetHandVel(Eigen::MatrixXd &handVel){
	handVel(0,0) = 10.0;
	handVel(1,0) = 0.0;
	handVel(2,0) = 0.0;
	handVel(3,0) = 0.0;
	handVel(4,0) = 0.0;
	handVel(5,0) = 0.0;
}

static const double TIME_LENGTH	= 1.0;
static const double TIME_SPAN	= 0.01;

//int main(void){
//	Trl::Logger lg("interface");
//	Trl::Coords armCoord(7);
//	Trl::IKCalculator ikc(7);
//
//	std::vector<double> xyz(3,0);
//	std::vector<double> curJ(7,0);
//
//	SetCurJointRad(curJ);
//
//	armCoord.InitCoords(curJ);
//	armCoord.GetFinger(xyz);
//	for(int i = 0; i < (int)xyz.size(); ++i) std::cout << i << " " << xyz[i] << std::endl;
//
//	//軌道点の設定
//	std::vector<double> viaPoint={xyz[0]-10,xyz[1],xyz[2]};//linearだと設定しても使われない
//	std::vector<double> endPoint={xyz[0],xyz[1],xyz[2]};
//
//	//ハンド姿勢の設定/////////////////////////////////////////////////
////	Trl::CherryCoords tc;
////	for(int i = 0; i < 3; ++i){ tc.middle(i,0) = endPoint[i];}
////	tc.top		= {tc.middle[0] + 10,	tc.middle[1] + 10,	tc.middle[2] + 10};
////	tc.bottom	= {tc.middle[0] + 10,	tc.middle[1] - 10,	tc.middle[2] + 10};
////	
//	std::vector<double> rpy(3,0);
//
//	rpy[0] = 0 * M_PI / 180;
//	rpy[1] = 0 * M_PI / 180;
//	rpy[2] = 30 * M_PI / 180;
/////////////////////////////////////////////////////////////////////////////
//
////////////IKの設定//////////////////////////////////////////////////////////
//	ikc.SetIKParam(Trl::basic, Trl::splineHand);
//	ikc.SetTimeParam(TIME_LENGTH,TIME_SPAN);
//	ikc.SetPathCoords(xyz,endPoint,viaPoint);
//	//ikc.SetPathCoords(xyz,endPoint);
//	//ikc.SetPostureParam(curJ,tc);
//	ikc.SetPostureParam(curJ,rpy);
//////////////////////////////////////////////////////////////////////////////
//
//	//返り値をもらう準備とか
//	std::vector<double> angleV(7,0);
//	//double trape = 0.0;
//
//	for(double i = 0; i <= TIME_LENGTH; i += TIME_SPAN){
//		//trape = TrapeInterpolate(M_PI/2, TIME_LENGTH, TIME_SPAN,i);
//	
//		//if(ikc.Run(curJ, i, angleV, trape)==false){
//		if(ikc.Run(curJ, i, angleV)==false){
//			std::cout << " program failed " << std::endl;
//		}
//
//		//ikc.Run(curJ, i, angleV, TIME_SPAN*M_PI);
//		lg.SetData(angleV,7);
//
//		for(int joint = 0; joint < 7; ++joint){
//			curJ[joint] += angleV[joint];
//		}
//	}
//
//	armCoord.InitCoords(curJ);
//	armCoord.GetFinger(xyz);
//
//	for(int i = 0; i < (int)xyz.size(); ++i) std::cout << i << " " << xyz[i] << std::endl;
//
//	lg.Write();
//
//	return 0;
//}

////trajectory test
//int main(void){
//	Trl::Logger lg("trajection");
//
//	Trl::Trajectory tr;
//
//	Trl::CherryCoords tc;
//
//	Eigen::Vector3d start;
//	Eigen::Vector3d via;
//	Eigen::Vector3d end;
//
//	start << 1,2,3;
//	via << 2,4,6;
//	end << 3,6,9;
//
//	tr.SetStart(start);
//	tr.SetVia(via);
//	tr.SetEnd(end);
//
//	tr.SetTimeParam(TIME_LENGTH, TIME_SPAN);
//
//	tr.GeneratePath();
//
//	Eigen::Vector3d ret = Eigen::Vector3d::Zero();
//
//	for(double i = 0; i < TIME_LENGTH; i+=TIME_SPAN){
//		tr.RunSpline(i,ret);
//		lg.SetData(ret);
//	}
//
//	lg.Write();
//}

//self motion test
//int main(void){
//	Trl::Logger lg("basicMotion");
//
//	Trl::SelfMotion bm(7);
//
//	Eigen::MatrixXd jr(7,1);
//	Eigen::MatrixXd vel(7,1);
//	Eigen::MatrixXd ret(7,1);
//
//	SetCurJointRad(jr);
//	SetHandVel(vel);
//
//	bm.Init(jr);
//
//	if(bm.Run(vel,ret)){
//		std::cout << ret << std::endl;
//		lg.SetData(ret);
//	}
//
//	lg.Write();
//
//	return 0;
//}

//basic motion test
//int main(void){
//	Trl::Logger lg("basicMotion");
//
//	Trl::BasicMotion bm(7);
//
//	Eigen::MatrixXd jr(7,1);
//	Eigen::MatrixXd vel(7,1);
//	Eigen::MatrixXd ret(7,1);
//
//	SetCurJointRad(jr);
//	SetHandVel(vel);
//
//
//	if(bm.Run(jr,vel,ret)){
//		std::cout << ret << std::endl;
//		lg.SetData(ret);
//	}
//
//
//	lg.Write();
//
//	return 0;
//}

//int main(void){
//	Trl::Logger hmrd("vel");
//	Trl::Logger s("integlar");
//
//	Eigen::MatrixXd jr = Eigen::MatrixXd(7,1);
//
//	Trl::HandMotion hm(7);
//
//	Trl::CherryCoords tc;
//	tc.top		= {565 + 10,	-18+10,	10};
//	tc.middle	= {565,		-18,	0};
//	tc.bottom	= {565 + 10,	-18-10,	-10};
//
////	std::vector<double> rpy(3,0);
////	rpy[0] = 0 * M_PI / 180;
////	rpy[1] = 0 * M_PI / 180;
////	rpy[2] = 60 * M_PI / 180;
//
//	SetCurJointRad(jr);
//
//	hm.SetTimeParam(TIME_LENGTH,TIME_SPAN);
//
//	hm.SetJointRad(jr);
//
//	hm.SetEndPosture(tc);
//
//	Eigen::Vector3d returnV = Eigen::Vector3d::Zero();
//
//	Eigen::Vector3d stack = Eigen::Vector3d::Zero();
//
//	for(double i = 0; i < 1; i += 0.01){
//		if(hm.Run(i,returnV) == true){
//			stack += returnV;
//			hmrd.SetData(returnV);
//			s.SetData(stack);
//		}
//	}
//	
//	hmrd.Write();
//	s.Write();
//
//	return 0;
//}


//int main(){
//	Eigen::Quaterniond s;
//	Eigen::Quaterniond e;
//
//	Trl::Logger lg("quaternion");
//
//	Eigen::MatrixXd jr = Eigen::MatrixXd(7,1);
//	SetCurJointRad(jr);
//
//	s = Trl::StartPosture(jr);
//
//	Trl::CherryCoords tc;
//	tc.top		= {10,	10,	0};
//	tc.middle	= {10,	0,	0};
//	tc.bottom	= {10,	-10,	0};
//
//	Eigen::Vector3d rpy;
//	rpy(0) = 0 * M_PI / 180;
//	rpy(1) = 0 * M_PI / 180;
//	rpy(2) = 10 * M_PI / 180;
//
//	e = Trl::EndPosture(rpy,jr);
//	//e = Trl::EndPosture(tc);
//
//	Trl::HandVelocity hv;
//
//	hv.SetTimeParam(TIME_LENGTH,TIME_SPAN);
//
//	hv.SetStart(s);
//	hv.SetEnd(e);
//
//	for(double i = 0; i < TIME_LENGTH; i += TIME_SPAN){
//		Eigen::Vector3d ret;
//
//		hv.Run(i,ret);
//
//		lg.SetData(ret);
//		
//	}
//
//	lg.Write();
//
//	return 0;
//}

//int main(void){
//	Trl::VelGenerator hg(7);
//	HandPathT hp;
//
//	Trl::Logger lg("velGene");
//	
//	// current joint
//	Eigen::MatrixXd jr = Eigen::MatrixXd(7,1);
//	SetCurJointRad(jr);
//
//	// target
//	Eigen::Vector3d target;
//
//	target(0) = 565;
//	target(1) = -18;
//	target(2) = 0;
//	
//	// via
//	Eigen::Vector3d via;
//
//	via(0) = 565;
//	via(1) = -20;
//	via(2) = 0;
//
//
//	// rpy value
//	Eigen::Vector3d rpy;
//	rpy(0) = 0 * M_PI / 180;
//	rpy(1) = 0 * M_PI / 180;
//	rpy(2) = 10 * M_PI / 180;
//
//	Trl::CherryCoords tc;
//	tc.top		= {10,	10,	10};
//	tc.middle	= {10,	0,	0};
//	tc.bottom	= {10,	-10,	-10};
//
//	hg.Straight(jr,target,Trl::EndPosture(rpy,jr),1.0,0.01,hp);
//	//hg.Spline(jr,target,via,Trl::EndPosture(tc),1.0,0.01,hp);
//
//	//std::cout << hp << std::endl;
//
//	lg.SetData(hp);
//
//	lg.Write();
//
//}

int main(void){

	Trl::Coords cd(7);
	Eigen::Vector3d fing;


	Trl::HTM obj(7);
	Eigen::MatrixXd jr = Eigen::MatrixXd(7,1);
	SetCurJointRad(jr);

	cd.InitCoords(jr);

	cd.GetFinger(fing);

	std::cout << fing << std::endl;

	obj.SetArmLength(Trl::kALength);
	obj.SetOffsetParam(Trl::kDLength);
	obj.SetAlphaParam(Trl::kAlphaRad);

	obj.CalcHTM(jr);

	obj.GetHTMAll();

	for(int i = 0; i < 8; ++i){
		std::cout << obj.om[i] << std::endl;
	}
}
