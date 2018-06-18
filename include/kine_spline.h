// 2017/05/06
// written by eiki obara
// reference http://www5d.biglobe.ne.jp/stssk/maze/spline.html

#ifndef __SPLINE_CLASS_H__
#define __SPLINE_CLASS_H__

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace Trl{

const int MAX_SPLINE_POINT = 3;

class Spline{
private:
	Eigen::Matrix<double,MAX_SPLINE_POINT,1> a;
	Eigen::Matrix<double,MAX_SPLINE_POINT,1> b;
	Eigen::Matrix<double,MAX_SPLINE_POINT,1> c;
	Eigen::Matrix<double,MAX_SPLINE_POINT,1> d;
	int num;
public:
	Spline();
	~Spline();
	void Init(Eigen::Vector3d &point);
	double Run(double t);
};


Spline::Spline() {
	a = b = c = d = Eigen::Matrix<double,MAX_SPLINE_POINT,1>::Zero();
	num = 0;
}

Spline::~Spline() {
}

void Spline::Init(Eigen::Matrix<double,MAX_SPLINE_POINT,1> &point){
	Eigen::Matrix<double,MAX_SPLINE_POINT+1,1> w;
	double temp;

	num = point.size() - 1;

	for(int i = 0;i <= num; ++i) a(i) = point(i);

	c(0) = c(num) = 0.0;

	for(int i = 1; i < num; ++i) c(i) = 3.0 *(a(i-1) - 2.0*a(i) + a(i+1));

	w(0) = 0.0;

	for(int i = 1; i < num; ++i){
		temp = 4.0 - w(i-1);
		c(i) = (c(i) - c(i-1)) / temp;
		w(i) = 1.0 / temp;
	}

	for(int i = num - 1; i > 0; --i) c(i) = c(i) - c(i+1) * w(i);

	b(num) = d(num) = 0.0;

	for(int i = 0; i < num; ++i){
		d(i) = (c(i+1) - c(i)) / 3.0;
		b(i) = a(i+1) - a(i) - c(i) - d(i);
	}
}

double Spline::Run(double t){
	double dt = 0;

	int i = (int)floor(t);

	if(i < 0){
		i = 0;
	}
	else if(i >= num){
		i = num - 1;
	}

	dt = t - (double)i;

	double buf = a(i) + (b(i) + (c(i) + d(i) * dt) * dt) * dt;

	return buf;
}


}	//namespace Trl

#endif // __SPLINE_CLASS_H__
