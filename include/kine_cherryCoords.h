// 2017/04/28
// author : eiki obara

#ifndef __HAND_POSTURE_H__
#define __HAND_POSTURE_H__

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <iostream>

namespace Trl{

struct CherryCoords{
public:
	Eigen::Vector3d top;
	Eigen::Vector3d middle;
	Eigen::Vector3d bottom;

	CherryCoords();
	~CherryCoords();

	void Display(void);

	void GetGraspDirection(Eigen::Vector3d &ret);
};

CherryCoords::CherryCoords() {
	top = Eigen::Vector3d::Zero();
	middle = Eigen::Vector3d::Zero();
	bottom = Eigen::Vector3d::Zero();
}

CherryCoords::~CherryCoords() {}

void CherryCoords::Display() {
    std::cout << "target coordinates : Mid" << std::endl;
    std::cout << middle << std::endl;

    std::cout << "target coordinates : Top" << std::endl;
    std::cout << top << std::endl;
    
    std::cout << "target coordinates : Bottom" << std::endl;
    std::cout << bottom << std::endl;
}

//把持方向決定
void CherryCoords::GetGraspDirection(Eigen::Vector3d &ret){
	Eigen::Vector3d top2midV = middle - top;
	Eigen::Vector3d top2btmV = bottom - top;
	
	top2midV.normalize();
	top2btmV.normalize();

	int sameVectorFlag = 0;

	for(int i = 0; i < 3; ++i){
		if(top2midV(i) == top2btmV(i))	++sameVectorFlag;
	}

	if(sameVectorFlag == 3){
		ret(0) = 1.0;
		ret(1) = 0.0;
		ret(2) = 0.0;

		return;
	}else{
	
		Eigen::Vector3d crossBuf = top2midV.cross(top2btmV);
	
		crossBuf.normalize();

		ret = crossBuf.cross(top2btmV);
	
		ret.normalize();

		int errorFlag = 0;

		for(int i = 0; i < 3; ++i){
			if(std::isnan(ret(i)))	++errorFlag;
			if(std::isinf(ret(i)))	++errorFlag;
		}

		if(errorFlag > 0){
			std::cout <<"GraspDireacion :cannot calculate"<< std::endl;
			std::cout <<"return middle point"<<std::endl;

			Eigen::Vector3d temp = Eigen::Vector3d::Zero();

			temp = bottom + ((top - bottom) / 2);
			ret = temp - middle;

			ret.normalize();
		}

		if(ret(0) < 0){
			for(int i = 0; i < 3; ++i){
				ret(i) = -1 * ret(i);
			}
		}

		return;
	}
}

}	//namespace Trl

#endif // !__HAND_POSTURE_H__
