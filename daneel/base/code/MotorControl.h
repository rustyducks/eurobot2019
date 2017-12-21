/*
 * BaseMotorControl.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef MOTORS_CONTOL_MOTORCONTROL_H_
#define MOTORS_CONTOL_MOTORCONTROL_H_
#include <Odometry.h>

#define INT_CLAMP 2000

//#include "arm_math.h"

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

	const arm_matrix_instance_f32* getTargetSpeed() const {
		return _targetSpeed;
	}

	/**
	 * \brief Set target speed
	 * \param targetSpeed The target speed in mm/s, in the table reference.
	 */
	void setTargetSpeed(arm_matrix_instance_f32* speed) {
		memcpy(_targetSpeed->pData, speed->pData, 3*sizeof(float32_t));
	}

	void setTargetSpeed(float32_t vx, float32_t vy, float32_t w) {
		_targetSpeed->pData[0] = vx;
		_targetSpeed->pData[1] = vy;
		_targetSpeed->pData[2] = w;
	}

	void setMotorCommand(int command, int pwmPin, int dirPin);

protected:

	//! Target speed : the speed at which the robot must be in m/s, in the table reference.
	arm_matrix_instance_f32* _targetSpeed;
	float32_t _intError[3];
	float32_t _prevError[3];

	float32_t KP[3];
	float32_t KI[3];
	float32_t KD[3];

	int prev_cons[3];

	arm_matrix_instance_f32* _intSpeedError;

};

extern MotorControl motorControl;

#endif /* MOTORS_CONTOL_MOTORCONTROL_H_ */
