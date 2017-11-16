/*
 * BaseMotorControl.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef MOTORS_CONTOL_BASEMOTORCONTROL_H_
#define MOTORS_CONTOL_BASEMOTORCONTROL_H_

class BaseMotorControl {
public:
	BaseMotorControl();
	virtual ~BaseMotorControl();

	/**
	 * \brief Do the necessary hardware and software initialisations.
	 */
	virtual void init();

	/**
	 * \brief Set speed target
	 * \param speed The target speed
	 */
	void setSpeed(float speed);

private:

	/**
	 *
	 */
	float _targetSpeed;
};

#endif /* MOTORS_CONTOL_BASEMOTORCONTROL_H_ */
