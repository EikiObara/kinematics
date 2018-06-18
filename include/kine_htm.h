// 2017/09/25
// author : eiki obara
//参考：ロボット制御基礎論　吉川恒夫　

#ifndef __KINE_HTM_H__
#define __KINE_HTM_H__

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include <iostream>

const double kCompareZero = 1.0e-06;

namespace Trl{

//static const double kBaseArmLength	= 164.0;
//static const double kUpperArmLength	= 322.0;
//static const double kForeArmLength	= 257.0;
//static const double kHandArmLength	= 154.0;

//homogeneous translation matrix
class HTM{
private:
	Eigen::MatrixXd alpha;
	Eigen::MatrixXd aLength;
	Eigen::MatrixXd dLength;

	int elemNum;
public:
	std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> om;
	std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> htm;

	//constractor
	HTM(int maxJoint);
	//destractor
	~HTM();

	int GetElem();

	//wrote in ref book (a_i) pp.27
	void SetArmLength(Eigen::MatrixXd _aLength);
	//wrote in ref book (d_i) pp.28
	void SetOffsetParam(Eigen::MatrixXd _offset);
	//wrote in ref book (alpha_i)
	void SetAlphaParam(Eigen::MatrixXd _alpha);

	//constractor overload
	void CalcHTM(Eigen::MatrixXd &curJointRad);
	//同時変換行列の算出
	void GetHTMAll(void);
	//start~end区間での同時変換行列を返す
	void GetHTM(const int start, const int end, Eigen::Matrix4d &ret);
};

HTM::HTM(int _elemNum){
	elemNum = _elemNum;

	alpha	= Eigen::MatrixXd(_elemNum+1,1);
	aLength = Eigen::MatrixXd(_elemNum+1,1);
	dLength = Eigen::MatrixXd(_elemNum+1,1);

	om = std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>(_elemNum+1);

	htm= std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>(_elemNum+1);

	for(int i = 0; i < _elemNum+1; ++i){
		om[i] = Eigen::Matrix4d::Identity();
	}
}

HTM::~HTM(){
}

int HTM::GetElem(){
	return elemNum;
}

void HTM::SetArmLength(Eigen::MatrixXd _dLength){
	dLength = _dLength;
	//for(int i = 0; i < _aLength.size(); ++i)std::cout << aLength(i,0) << std::endl;
}

void HTM::SetAlphaParam(Eigen::MatrixXd _alpha){
	alpha = _alpha;
	//for(int i = 0; i < _alpha.size(); ++i)	std::cout << alpha(i,0) << std::endl;
}

void HTM::SetOffsetParam(Eigen::MatrixXd _offset){
	aLength = _offset;
	//for(int i = 0; i < _offset.size(); ++i)	std::cout << dLength(i,0) << std::endl;
}

//end effector 分でelemnumが1増える
void HTM::CalcHTM(Eigen::MatrixXd &curJointRad){
	for(int joint = 0; joint < elemNum+1; ++joint){
		double cos_t, sin_t, cos_a, sin_a;

		if(joint != 7){
			cos_t = cos(curJointRad(joint));
			sin_t = sin(curJointRad(joint));
			cos_a = cos(alpha(joint));
			sin_a = sin(alpha(joint));
		}else if(joint == 7){
			cos_t = cos(0.0);
			sin_t = sin(0.0);
			cos_a = cos(0.0);
			sin_a = sin(0.0);
		}

		om[joint](0,0) = cos_t;
		om[joint](0,1) = -sin_t;
		om[joint](0,2) = 0.0;
		om[joint](0,3) = aLength(joint,0);

		om[joint](1,0) = cos_a * sin_t;
		om[joint](1,1) = cos_a * cos_t;
		om[joint](1,2) = -sin_a;
		om[joint](1,3) = -sin_a * dLength(joint,0);

		om[joint](2,0) = sin_a * sin_t;
		om[joint](2,1) = sin_a * cos_t;
		om[joint](2,2) = cos_a;
		om[joint](2,3) = cos_a * dLength(joint,0);

		om[joint](3,0) = 0.0;
		om[joint](3,1) = 0.0;
		om[joint](3,2) = 0.0;
		om[joint](3,3) = 1.0;
	}

	for(int i = 0; i < elemNum; ++i){
		for(int j = 0; j < om[i].rows(); ++j){
			for(int k = 0; k < om[i].cols(); ++k){
				if(fabs(om[i](j,k)) < kCompareZero){
					om[i](j,k) = 0.0;
				}
			}
		}
	}
}

void HTM::GetHTMAll(void){
	Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();

	for(int i = 0; i < elemNum+1; ++i){
		htm[i] = temp * om[i];
		temp = htm[i];
	}
}

void HTM::GetHTM(const int start, const int end, Eigen::Matrix4d &ret){
	Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();

	for(int joint = start; joint <= end; ++joint){
		temp *= om[joint];
	}

	ret = temp;
}

}	//namespace Trl

#endif // __KINE_HTM_H__
