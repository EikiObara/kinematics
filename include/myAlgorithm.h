#ifndef __MY_ALGORITHM_H__
#define __MY_ALGORITHM_H__

#include "Eigen/Core"

unsigned int FindMinimum(Eigen::MatrixXd &target, Eigen::MatrixXd ratio){
	unsigned int ret = 0;
	double temp = 1;

	for(unsigned int i = 0; i < target.rows(); ++i){
		if(target(i,0) < 0 && target(i,0) < temp){
			temp = target(i,0) / ratio(i,0);
			ret = i;
		}
	}
	return ret;
}

#endif //__MY_ALGORITHM_H__
