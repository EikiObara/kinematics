// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __TRAPEZOIDAL_INTERPOLATION_H__
#define __TRAPEZOIDAL_INTERPOLATION_H__

double TrapeInterpolate(const double moveLen, const double timeLen, const double timeSpan ,const double curTime) {
	double buf = 0;

	double accelTime = timeLen / 4;

	if (curTime < accelTime) {
		buf = timeSpan * (curTime / accelTime) * (moveLen /(timeLen - accelTime));
	}
	else if (curTime >= accelTime && curTime <= (timeLen - accelTime)) {
		buf = timeSpan * ((moveLen) / (timeLen - accelTime));
	}
	else if (curTime >(timeLen - accelTime) && curTime <= timeLen) {
		buf = timeSpan * (moveLen * (timeLen - curTime)) / ((timeLen - accelTime) * accelTime);
	}
	else if(curTime > timeLen){
		buf = 0.0;
	}

	return buf;
}
#endif //__TRAPEZOIDAL_INTERPOLATION_H__
