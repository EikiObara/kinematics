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

void CreateHTM(double jointRad, double alphaRad,
	double armLength, double armOffset, Eigen::Matrix4d &ret){

	double cos_t, sin_t, cos_a, sin_a;

	cos_t = cos(jointRad);
	sin_t = sin(jointRad);
	cos_a = cos(alphaRad);
	sin_a = sin(alphaRad);

	ret(0,0) = cos_t;
	ret(0,1) = -sin_t;
	ret(0,2) = 0.0;
	ret(0,3) = armOffset;

	ret(1,0) = cos_a * sin_t;
	ret(1,1) = cos_a * cos_t;
	ret(1,2) = -sin_a;
	ret(1,3) = -sin_a * armLength;

	ret(2,0) = sin_a * sin_t;
	ret(2,1) = sin_a * cos_t;
	ret(2,2) = cos_a;
	ret(2,3) = cos_a * armLength;

	ret(3,0) = 0.0;
	ret(3,1) = 0.0;
	ret(3,2) = 0.0;
	ret(3,3) = 1.0;
}

//homogeneous translation matrix
class HTM{
private:
	Eigen::MatrixXd alpha;
	Eigen::MatrixXd offset;
	Eigen::MatrixXd armLength;

	int elemNum;
	int linkNum;
public:
	std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> om;
	std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> htm;

	//constractor
	HTM(int _elemNum);
	//destractor
	~HTM();

	int GetElem();

	//wrote in ref book (a_i) pp.27
	void SetArmLength(Eigen::MatrixXd _armLength);
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
	linkNum = _elemNum + 1;

	offset = Eigen::MatrixXd::Zero(linkNum,1); 
	armLength = Eigen::MatrixXd::Zero(linkNum,1); 
	alpha	= Eigen::MatrixXd::Zero(linkNum,1);

	om = std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>(linkNum);
	htm= std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>(linkNum);

	for(int i = 0; i < linkNum; ++i)om[i] = Eigen::Matrix4d::Identity();
}

HTM::~HTM(){}

int HTM::GetElem(){
	return elemNum;
}

void HTM::SetArmLength(Eigen::MatrixXd _armLength){
	armLength = _armLength;
//	std::cout << armLength << std::endl;
//	std::cout << "rows" << armLength.rows() << std::endl;
//	std::cout << "cals" << armLength.cols() << std::endl;
}

void HTM::SetAlphaParam(Eigen::MatrixXd _alpha){
	alpha = _alpha;
//	std::cout << alpha << std::endl;
//	std::cout << "rows" << alpha.rows() << std::endl;
//	std::cout << "cals" << alpha.cols() << std::endl;
}

void HTM::SetOffsetParam(Eigen::MatrixXd _offset){
	offset = _offset;
//	std::cout << offset << std::endl;
//	std::cout << "rows" << offset.rows() << std::endl;
//	std::cout << "cals" << offset.cols() << std::endl;
}


//end effector 分でelemnumが1増える
void HTM::CalcHTM(Eigen::MatrixXd &curJointRad){

	for(int joint = 0; joint < linkNum; ++joint){

		if(joint != elemNum){
		CreateHTM(curJointRad(joint),alpha(joint),armLength(joint),offset(joint),om[joint]);
		}else if(joint == elemNum){
		CreateHTM(curJointRad(0.0),alpha(joint),armLength(joint),offset(joint),om[joint]);
		}
	}

	for(int i = 0; i < linkNum; ++i){
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

	for(int i = 0; i < linkNum; ++i){
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
