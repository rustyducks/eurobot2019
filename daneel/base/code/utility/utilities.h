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

arm_matrix_instance_f32* makeSpeed(float32_t vx, float32_t vy, float32_t w);
void freeSpeed(arm_matrix_instance_f32* speed);

inline float32_t RW_to_W(float32_t rw) {
	return rw/ROBOT_RADIUS;
}

inline float32_t W_to_RW(float32_t w) {
	return w * ROBOT_RADIUS;
}

#endif /* UTILITY_UTILITIES_H_ */
