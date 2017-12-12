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
	 * \brief Set robot speed from robot to table reference system
	 * \param speed Robot speed in the robot reference system
	 */
	void setSpeed(arm_matrix_instance_f32* speed);

	/**
	 * \return the sum of the moves from id originId to the most recent
	 */
	Move3D getMoveDelta(int originId);

	/**
	 * \brief rotate one step ahead the circular buffer
	 * Should be called just after getMoveDelta.
	 */
	void JumpToNextMove() {
		_readIndex = (_readIndex + 1) % MOVE_HISTORY_LENGHT;
	};


	void isr1();
	void isr11();
	void isr2();
	void isr22();
	void isr3();
	void isr33();

	arm_matrix_instance_f32* getMotorSpeeds() {
		return _motorSpeeds;
	}

protected:

	/**
	 * \return delta theta since last time moveDelta was reset.
	 * useful for estimating the current theta, in combination with _thetaAI
	 */
	float getDeltaTheta();

	/**
	 * \brief Adds move to the last move of _moveDelta (the current one)
	 */
	void addMove(Move3D move);

	/**
	 * \brief Moves between RAZ
	 *
	 * Circular buffer holding move deltas.
	 * The move at the current index is refined at each odometry update (high frequency).
	 * At low frequency, this move is sent to the AI
	 */
	Move3D _moveDelta[];

	int _readIndex;

	/**
	 *  Last computed speed of the robot
	 *  defined by (vx, vy, Rw)
	 */

	arm_matrix_instance_f32* _speed;
	//Speed3D _speed;

	//! Last theta from AI
	float _thetaAI;

	int _inc1, _inc2, _inc3;

	arm_matrix_instance_f32* _motorSpeeds;

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
