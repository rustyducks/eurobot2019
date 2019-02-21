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

	float get_speed_setpoint() {
		return _speed_setpoint;
	}
	float get_omega_setpoint() {
		return _omega_setpoint;
	}

	void set_speed_setpoint(float speed_setpoint) {
		_speed_setpoint = speed_setpoint;
	}

	void set_omega_setpoint(float omega_setpoint) {
		_omega_setpoint = omega_setpoint;
	}

	/**
	 * \brief Executes a control loop, trying to reach the target speed.
	 *
	 * This function should be called at fixed frequency.
	 */
	void control();


	void setMotorCommand(int command, int pwmPin, int dirPin);

	void reset();

protected:

	float _speed_setpoint;
	float _omega_setpoint;

	float32_t _intError_speed;
	float32_t _prevError_speed;
	float32_t _intError_omega;
	float32_t _prevError_omega;

	float32_t KP_SPEED;
	float32_t KI_SPEED;
	float32_t KD_SPEED;
	float32_t KP_OMEGA;
	float32_t KI_OMEGA;
	float32_t KD_OMEGA;

};

extern MotorControl motorControl;

#endif /* MOTORS_CONTOL_MOTORCONTROL_H_ */
