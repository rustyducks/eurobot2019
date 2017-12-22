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

const int MOVE_HISTORY_LENGHT = 10;

class Odometry {
public:
	Odometry();
	virtual ~Odometry();

	void init();

	/**
	 * \brief Updates move and speed by reading increments number.
	 *
	 * This function should be called at fixed frequency, before the motor control.
	 */
	void update();

	/**
	 * \brief Update method for trike configuration.
	 */
	void updateTrike();

	/**
	 * \brief Update method for differential configuration.
	 */
	void updateDifferential();

	/**
	 * \brief Update method for holonomic configuration.
	 */
	void updateHolonomic();


	/**
	 * \brief return robot speed in the table reference system
	 */
	arm_matrix_instance_f32* getSpeed() {
		return _speed;
	}

	/**
	 * \return the sum of the moves since last time it was sent (and reset).
	 */
	arm_matrix_instance_f32* getMoveDelta() {
		return _moveDelta;
	}

	void isr1();
	void isr11();
	void isr2();
	void isr22();
	void isr3();
	void isr33();

	arm_matrix_instance_f32* getMotorSpeeds() {
		return _motorSpeeds;
	}

	/**
	 * \return delta theta since last time moveDelta was reset.
	 * useful for estimating the current theta, in combination with _thetaAI
	 */
	float getDeltaTheta();

	void resetMoveDelta();

	void recalerTheta(float32_t theta);

	float32_t getTheta() const {
		return _theta;
	}

	void reset();

protected:

	/**
	 * \brief Adds move to the last move of _moveDelta (the current one)
	 */
	void updatePosition();

	float32_t _theta;		//true theta, updated

	arm_matrix_instance_f32* _moveDelta;

	int _inc1, _inc2, _inc3;

	//!
	arm_matrix_instance_f32* _motorsDisplacement;
	arm_matrix_instance_f32* _robotDisplacement;

	arm_matrix_instance_f32* _motorSpeeds;

	/**
	 *  Last computed speed of the robot
	 *  defined by (vx, vy, Rw)
	 */
	arm_matrix_instance_f32* _speed;

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
void ISR3();
void ISR33();

#endif /* ODOMETRY_ODOMETRY_H_ */
