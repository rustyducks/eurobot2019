/*
 * BaseMotorControl.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef MOTORS_CONTOL_MOTORCONTROL_H_
#define MOTORS_CONTOL_MOTORCONTROL_H_
#include <Odometry.h>
#include <Speed3D.h>

/**
 * \brief Base class for every motor control classes.
 */
class MotorControl {
public:
	MotorControl();
	virtual ~MotorControl() {};

	/**
	 * \brief Do the necessary hardware and software initialisations.
	 */
	void init();

	/**
	 * \brief Executes a control loop, trying to reach the target speed.
	 *
	 * This function should be called at fixed frequency.
	 */
	void control();

	/**
	 * \brief Control loop for trike configuration
	 */
	void controlTrike();

	/**
	 * \brief Control loop for differential configuration
	 */
	void controlDifferential();

	/**
	 * \brief Control loop for holonomic configuration
	 */
	void controlHolonomic();

	const Speed3D& getTargetSpeed() const {
		return _targetSpeed;
	}

	/**
	 * \brief Set target speed
	 * \param targetSpeed The target speed in m/s, in the table reference.
	 */
	void setTargetSpeed(const Speed3D& targetSpeed) {
		_targetSpeed = targetSpeed;
	}

protected:

	//! Target speed : the speed at which the robot must be in m/s, in the table reference.
	Speed3D _targetSpeed;

	float _intSpeedError;
	float _intHeadingError;

};

extern MotorControl motorControl;

#endif /* MOTORS_CONTOL_MOTORCONTROL_H_ */
