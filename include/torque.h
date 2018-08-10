
#ifndef __TORQUE_H__
#define __TORQUE_H__

#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"

const std::vector<double> radius = {0,0.040,0.040,0.025,0.025,0.025};

Eigen::MatrixXd GetPulleyMatrix(){
	Eigen::MatrixXd mat(6,7);

	for(int i = 0; i < 6; ++i){
		for(int j = 0; j < 7; ++j)	mat(i,j) = radius[i];
	}

	mat(0,0) = 1;	
	mat(1,0) = 0;	mat(1,1) *= -1;	
	mat(2,0) = 0;	mat(2,1) = 0;	mat(2,2) *= -1;
	mat(3,0) = 0;	mat(3,1) = 0;	mat(3,2) = 0;	mat(3,3) *= 1;	mat(3,4) *= -1;	mat(3,5) *= -1;	mat(3,6) *= 1;
	mat(4,0) = 0;	mat(4,1) = 0;	mat(4,2) = 0;	mat(4,3) *= 1;	mat(4,4) *= -1;	mat(4,5) *= 1;	mat(4,6) *= -1;
	mat(5,0) = 0;	mat(5,1) = 0;	mat(5,2) = 0;	mat(5,3) *= 1;	mat(5,4) *= 1;	mat(5,5) *= -1;	mat(5,6) *= -1;

	return mat;
}

Eigen::MatrixXd GetPseudoMatrix(Eigen::MatrixXd &origin){
	Eigen::MatrixXd diagonal = origin * origin.transpose();
	Eigen::MatrixXd inv = origin.transpose() * diagonal.inverse();
	return inv;
}

#endif //__TORQUE_H__
