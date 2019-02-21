/*
 * BaseHolonomy.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef ODOMETRY_ODOMETRY_H_
#define ODOMETRY_ODOMETRY_H_
#include <Move3D.h>
#include "arm_math.h"
#include "params.h"

const int MOVE_HISTORY_LENGHT = 10;

class Odometry {
public:
	Odometry();
	virtual ~Odometry();

	void init();
	void reset();

	/**
	 * \brief Updates move and speed by reading increments number.
	 *
	 * This function should be called at fixed frequency, before the motor control.
	 */
	void update();

	void isr1();
	void isr11();
	void isr2();
	void isr22();

	void set_pos(float x, float y, float theta) {
		_x = x;
		_y = y;
		_theta = theta;
	}

	float get_pos_x() {
		return _x;
	}

	float get_pos_y() {
		return _y;
	}

	float get_pos_theta()
	{
		return _theta;  //TODO normalize angle
	}

	float get_speed() {
		return _speed;
	}

	float get_omega() {
		return _omega;
	}

protected:

	volatile int _incr1, _incr2;

	float _x, _y, _theta;

	float _speed, _omega;

};


extern Odometry odometry;

/**
 * Instanciate the Odometry class, then configure corrects interruption routines
 */
void initOdometry();

void ISR1();
void ISR11();
void ISR2();
void ISR22();

#endif /* ODOMETRY_ODOMETRY_H_ */
