//2017/09/25
// author: eiki obara


#ifndef __KINE_BEZIER_H__
#define __KINE_BEZIER_H__

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace Trl{

const int MAX_NODE = 3;

class BSpline{
private:
	Eigen::Matrix<double,MAX_NODE,1> nodes;
public:
	BSpline();
	~BSpline();

	void Init(Eigen::Matrix<double,MAX_NODE,1> &_nodes);
	double Run(double t);
};

int Combination(unsigned int n, unsigned int r){
	if(r * 2 > n) r = n - r;

	int dividend = 1;
	int divisor = 1;
	for(unsigned int i = 1; i <= r; ++i){
		dividend *= (n - i + 1);
		divisor *= i;
	}

	return dividend / divisor;
}

BSpline::BSpline(){
	nodes = Eigen::Matrix<double,MAX_NODE,1>::Zero();
}

BSpline::~BSpline(){}

void BSpline::Init(Eigen::Matrix<double,MAX_NODE,1> &_nodes){

	for(int i = 0; i < MAX_NODE; ++i){
		nodes(i) = _nodes(i);
	}
}

double BSpline::Run(double t){
	double sum = 0;
	double c1 = 0;
	double c2 = 0;

	int numOfTimes = MAX_NODE - 1;

	for(unsigned int r = 0; r <= numOfTimes - r; ++r){
		c1 = pow(1 - t, numOfTimes - r);
		c2 = pow(t, r);
		sum += Combination(numOfTimes, r) * c1 * c2 * nodes(r);
	}

	return sum;
}

}	//namespace Trl

#endif //__KINE_BEZIER_H__
