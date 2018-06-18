// 2017/04/28
// author : by eiki obara

#ifndef __CONVERTOR_H__
#define __CONVERTOR_H__

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace Trl{

const double kCompareZero = 1.0e-06;

void Euler2Angular(Eigen::Vector3d &nowEuler,Eigen::Vector3d &eulerVel,Eigen::Vector3d &angleVel){
	Eigen::Matrix3d euler2angulerM = Eigen::Matrix3d::Zero();
	
	
	euler2angulerM(0,0) = 0.0;
	euler2angulerM(0,1) = -sin(nowEuler(0,0));
	euler2angulerM(0,2) = cos(nowEuler(0,0)) * sin(nowEuler(1,0));

	euler2angulerM(1,0) = 0.0;
	euler2angulerM(1,1) = cos(nowEuler(0,0));
	euler2angulerM(1,2) = sin(nowEuler(0,0)) * sin(nowEuler(1,0));

	euler2angulerM(2,0) = 1.0;
	euler2angulerM(2,1) = 0.0;
	euler2angulerM(2,2) = cos(nowEuler(1,0));

	//std::cout << "e2a mat\n" << euler2angulerM << std::endl;
	//std::cout << "eulerVel\n" << eulerVel << std::endl;

	angleVel = euler2angulerM * eulerVel;

	//std::cout << "anglevel\n" << angleVel << std::endl;
}

void DirectVec2RotMat(Eigen::Vector3d &directionX, Eigen::Vector3d &directionZ, Eigen::Matrix4d &ret){
	directionX.normalize();
	directionZ.normalize();

	Eigen::Vector3d directionY = Eigen::Vector3d::Zero();

	directionY = directionZ.cross(directionX);

	directionY.normalize();

	ret = Eigen::Matrix4d::Zero();

	for(int i = 0; i < 3; ++i){
		ret(i,0) = directionX(i);
		ret(i,1) = directionY(i);
		ret(i,2) = directionZ(i);
	}

	ret(3,3) = 1.0;

	for(int i = 0; i < 4; ++i){
		for(int j = 0; j < 4; ++j){
			if(fabs(ret(i,j)) < kCompareZero){
				ret(i,j) = 0.0;
			}
			if(ret(i,j) == -0.0){
				ret(i,j) = 0.0;
			}
		}
	}
}

//////////////////
//roll pitch yaw//
//////////////////

void RPY2RotMat(Eigen::Vector3d &rpy, Eigen::Matrix4d &mat){
	double cPhi = cos(rpy(0));
	double sPhi = sin(rpy(0));
	double cTheta = cos(rpy(1));
	double sTheta = sin(rpy(1));
	double cPsi = cos(rpy(2));
	double sPsi = sin(rpy(2));

	mat = Eigen::Matrix4d::Zero();

	mat(0,0) = cPhi * cTheta;
	mat(0,1) = cPhi * sTheta * sPsi - sPhi * cPsi;
	mat(0,2) = cPhi * sTheta * cPsi + sPhi * sPsi;
	mat(0,3) = 0.0;

	mat(1,0) = sPhi * cTheta;
	mat(1,1) = sPhi * sTheta * sPsi + cPhi * cPsi;
	mat(1,2) = sPhi * sTheta * cPsi - cPhi * sPsi;
	mat(1,3) = 0.0;

	mat(2,0) = -sTheta;
	mat(2,1) = cTheta * sPsi;
	mat(2,2) = cTheta * cPsi;
	mat(2,3) = 0.0;

	mat(3,0) = 0.0;
	mat(3,1) = 0.0;
	mat(3,2) = 0.0; 
	mat(3,3) = 1.0;
}

//void RPY2RotMat(Eigen::Vector3d &rpy, Eigen::Matrix4d &mat){
//	using namespace Eigen;
//
//	Matrix3f bufMat;
//
//	bufMat = AngleAxisf(rpy[0],Vector3f::UnitX())
//	* AngleAxisf(rpy[1], Vector3f::UnitY())
//	* AngleAxisf(rpy[2], Vector3f::UnitZ());
//
//	for(int i = 0; i < 3; ++i){
//		for(int j = 0; j < 3; ++j){
//			mat(i,j) = bufMat(i,j);
//		}
//	}
//}





void RotMat2RPY(Eigen::Matrix4d &mat, Eigen::Vector3d &rpy){
	if(mat(2,0) > 0.98){
		rpy(0) = atan2(mat(1,0), mat(0,0));
		rpy(1) = M_PI / 2;
		rpy(2) = 0.0;
		return;
	}
	if(mat(2,0) < -0.98){
		rpy(0) = atan2(mat(1,0), mat(0,0));
		rpy(1) = -M_PI / 2;
		rpy(2) = 0.0;
		return;
	}

	rpy(0) = atan2(mat(1,0), mat(0,0));
	rpy(1) = asin(-mat(2,0));
	rpy(2) = atan2(mat(2,1), mat(2,2));
	return;
}

//////////////
//quaternion//
//////////////

void Quat2RotMat(Eigen::Quaterniond &q, Eigen::Matrix4d &m){
	q.normalize();

	double xx = q.x() * q.x();
	double yy = q.y() * q.y();
	double zz = q.z() * q.z();
	double ww = q.w() * q.w();
	double xy = q.x() * q.y();
	double yz = q.y() * q.z();
	double zx = q.z() * q.x();
	double xw = q.x() * q.w();
	double yw = q.y() * q.w();
	double zw = q.z() * q.w();

	double invs = 1/ (xx + yy + zz + ww);

	m(0,0) = 1.0 - 2 * (yy + zz) * invs;
	m(0,1) = 2.0 * (xy - zw) * invs;
	m(0,2) = 2.0 * (zx + yw) * invs;
	m(0,3) = 0.0;
	
	m(1,0) = 2.0 * (xy + zw) * invs;
	m(1,1) = 1.0 - 2 * (xx + zz) * invs;
	m(1,2) = 2.0 * (yz - xw) * invs;
	m(1,3) = 0.0;

	m(2,0) = 2.0 * (zx - yw) * invs;
	m(2,1) = 2.0 * (yz + xw) * invs;
	m(2,2) = 1.0 - 2 * (xx + yy) * invs;
	m(2,3) = 0.0;

	m(3,0) = 0.0;
	m(3,1) = 0.0;
	m(3,2) = 0.0;
	m(3,3) = 1.0;
}

void RotMat2Quat(Eigen::Matrix4d &m, Eigen::Quaterniond &q){

	double diagonal = m(0,0)+m(1,1)+m(2,2);
	
	if(diagonal > 0){
		double s = 0.5 / pow(diagonal + 1.0, 0.5);
		q.w() = 0.25 / s;
		q.x() = (m(2,1) - m(1,2)) * s;
		q.y() = (m(0,2) - m(2,0)) * s;
		q.z() = (m(1,0) - m(0,1)) * s;
	}else{
		if(m(0,0) > m(1,1) && m(0,0) > m(2,2)){
			double s = 2.0 * pow(1.0 + m(0,0) -m(1,1) -m(2,2), 0.5);

			q.w() = (m(2,1) - m(1,2)) / s;
			q.x() = 0.25 * s;
			q.y() = (m(0,1) + m(1,0)) / s;
			q.z() = (m(1,2) + m(2,1)) / s;
		}else if(m(1,1) > m(2,2)){
			double s = 2.0 * pow(1.0 - m(0,0) + m(1,1) -m(2,2), 0.5);

			q.w() = (m(0,2) - m(2,0)) / s;
			q.x() = (m(0,1) + m(1,0)) / s;
			q.y() = 0.25 * s;
			q.z() = (m(1,2) + m(2,1)) / s;
		}else{
			double s = 2.0 * pow(1.0 - m(0,0) -m(1,1) + m(2,2), 0.5);
			q.w() = (m(1,0) - m(0,1)) / s;
			q.x() = (m(0,2) + m(2,0)) / s;
			q.y() = (m(1,2) + m(2,1)) / s;
			q.z() = 0.25 * s;
		}
	}
}

///////////////////
//rotation matrix//
///////////////////

void RotMat2EulerZXZ(Eigen::Matrix4d &m, Eigen::Vector3d &e){
	e(0) = atan2(m(0,2), m(1,2));
	e(1) = acos(m(2,2));
	e(2) = atan2(m(2,0), -m(2,1));
}

void RotMat2EulerZYX(Eigen::Matrix4d &m, Eigen::Vector3d &e){
	if(m(0,2) < -0.998){
		//std::cout << "m(0,2) < -0.998" << std::endl;
		e(0) = M_PI / 2;
		e(1) = 0;
		e(2) = e(0) + atan2(m(1,0),m(2,0));
		return;
	}
	else if(m(0,2) > 0.998){
		//std::cout << "m(0,2) > 0.998" << std::endl;
		e(0) = M_PI / 2;
		e(1) = 0;
		e(2) = -e(0) + atan2(-m(1,0),-m(2,0));
		return;
	}
	else{
		double x1 = -asin(m(0,2));
		double x2 = M_PI - x1;

		double y1 = atan2(m(1,2)/cos(x1),m(2,2)/cos(x1));
		double y2 = atan2(m(1,2)/cos(x2),m(2,2)/cos(x2));
		double z1 = atan2(m(0,1)/cos(x1),m(0,0)/cos(x1));
		double z2 = atan2(m(0,1)/cos(x2),m(0,0)/cos(x2));

		if(fabs(x1)+fabs(y1)+fabs(z1) <= fabs(x2)+fabs(y2)+fabs(z2)){
			e(0) = x1;
			e(1) = y1;
			e(2) = z1;
		}
		else{
			e(0) = x2;
			e(1) = y2;
			e(2) = z2;
		}
	}
}

void Quat2Euler(Eigen::Quaterniond &q, Eigen::Vector3d &e){
//	std::cout << "**************quat 2 euler***************" << std::endl;
	Eigen::Matrix4d bufMat = Eigen::Matrix4d::Zero();

//	std::cout << q.w() << std::endl;
//	std::cout << q.x() << std::endl;
//	std::cout << q.y() << std::endl;
//	std::cout << q.z() << std::endl;

	Quat2RotMat(q, bufMat);
//	std::cout << "quat 2 matrix\n" << bufMat << std::endl;

	RotMat2EulerZXZ(bufMat, e);
//	std::cout << "matrix to euler\n" << e << std::endl;
	
}

}	//namespace Trl

#endif // __CONVERTOR_H__
