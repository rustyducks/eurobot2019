/*
 * Simulator.h
 *
 *  Created on: 20 déc. 2017
 *      Author: fabien
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "params.h"
#define NB_MOTORS 2

class Simulator {
public:
	Simulator();
	virtual ~Simulator();

	int readEnc(int moteurNb);
	void setMotorCommand(int command, int motor);
	void update();

	void reset();

private:
	float vDiff[2];
	float acc[2];

	float vFinal[2];
	float v[2];

	//! time at which the motor reach about 63% of its final speed. 95% at 5*tau.
	float tau;

	float encs[2];

	//! factor between pwm command and robot speed (dependent of wheel size,motor, and battery level)
	float KMotor;
};

#ifdef SIMULATOR
extern Simulator simulator;
#endif

#endif /* SIMULATOR_H_ */
