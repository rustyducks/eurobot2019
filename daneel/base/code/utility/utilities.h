/*
 * utilities.h
 *
 *  Created on: 19 nov. 2017
 *      Author: fabien
 */

#ifndef UTILITY_UTILITIES_H_
#define UTILITY_UTILITIES_H_

#include "arm_math.h"
#include "params.h"

template<class T> constexpr const T& clamp(const T& lo,const T& v, const T& hi )
{
    return min(hi, max(lo, v));
}

/**
 * Centers an angle in radians to [-pi, pi[
 */
constexpr const float center_radians(float angle){
	while (angle >= M_PI){
		angle -= 2 * M_PI;
	}
	while (angle < -M_PI){
		angle += 2 * M_PI;
	}
	return angle;
}

#endif /* UTILITY_UTILITIES_H_ */
