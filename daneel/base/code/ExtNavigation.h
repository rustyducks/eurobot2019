/*
 * ExtNavigation.h
 *
 *  Created on: 12 déc. 2017
 *      Author: fabien
 */

#ifndef EXTNAVIGATION_H_
#define EXTNAVIGATION_H_

#include "arm_math.h"
#include "utilities.h"

class ExtNavigation {
public:
	ExtNavigation();
	virtual ~ExtNavigation();

	/**
	 * Set the new commands to the motor controller according to the current theta and
	 * speed consigne (table reference frame)
	 */
	void update();

	/**
	 * \brief Set the speed consigne from its 3 composantes.
	 * \param vx  speed along the X axis in mm/s
	 * \param vy  speed along the Y axis in mm/s
	 * \param w   rotation speed in rd/s
	 */
	void setSpeedCons(float32_t vx, float32_t w) {
		_speed_setpoint = vx;
		_omega_setpoint = w;
	}

	void reset();

private:
	float _omega_setpoint;
	float _speed_setpoint;
};

extern ExtNavigation extNavigation;

#endif /* EXTNAVIGATION_H_ */
