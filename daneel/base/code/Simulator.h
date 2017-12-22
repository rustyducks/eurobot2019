/*
 * Simulator.h
 *
 *  Created on: 20 déc. 2017
 *      Author: fabien
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "params.h"

class Simulator {
public:
	Simulator();
	virtual ~Simulator();

	int readEnc(int moteurNb);
	void setMotorCommand(int command, int motor);
	void update();

private:
	float vDiff[3];
	float acc[3];

	float vFinal[3];
	float v[3];

	//! time at which the motor reach about 63% of its final speed. 95% at 5*tau.
	float tau;

	float encs[3];

	//! factor between pwm command and robot speed (dependent of wheel size,motor, and battery level)
	float KMotor;
};

#ifdef SIMULATOR
extern Simulator simulator;
#endif

#endif /* SIMULATOR_H_ */
