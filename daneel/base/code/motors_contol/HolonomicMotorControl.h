/*
 * HolonomicMotorControl.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef MOTORS_CONTOL_HOLONOMICMOTORCONTROL_H_
#define MOTORS_CONTOL_HOLONOMICMOTORCONTROL_H_

#include "BaseMotorControl.h"

class HolonomicMotorControl: public BaseMotorControl {
public:
	HolonomicMotorControl();
	virtual ~HolonomicMotorControl();
};

#endif /* MOTORS_CONTOL_HOLONOMICMOTORCONTROL_H_ */
