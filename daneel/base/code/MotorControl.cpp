/*
 * BaseMotorControl.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <MotorControl.h>
#include "params.h"
#include "utilities.h"

#ifdef SIMULATOR
#include "Simulator.h"
#endif
//#include <DynamixelSerial5.h>

#define sig(x) (x) > 0 ? 1 : 0

MotorControl motorControl = MotorControl();

MotorControl::MotorControl() {
	_intSpeedError = makeSpeed(0,0,0);
	_targetSpeed = makeSpeed(0,0,0);
	 prev_cons[0] =  prev_cons[1] =  prev_cons[2] = 0;
	 _intError[0] =  _intError[1] =  _intError[2] = 0;
	_prevError[0] = _prevError[1] = _prevError[2] = 0;
	KP[0] = KP[1] = KP[2] = 0.3;			//0.4
	KI[0] = KI[1] = KI[2] = 0.2;
	KD[0] = KD[1] = KD[2] = 0.1;

}

void MotorControl::init() {

	analogWriteResolution(8);

#if defined(TRIKE)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	//pinMode(MOT1_ENCA, INPUT);
	//pinMode(MOT1_ENCB, INPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

#elif defined(DIFFERENTIAL)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	pinMode(MOT1_BRK, OUTPUT);
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

	pinMode(MOT2_PWM, OUTPUT);
	pinMode(MOT2_DIR, OUTPUT);
	pinMode(MOT2_BRK, OUTPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	analogWriteFrequency(MOT2_PWM, PWM_FREQUENCY);

#elif defined(HOLONOMIC)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	pinMode(MOT1_BRK, OUTPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

	pinMode(MOT2_PWM, OUTPUT);
	pinMode(MOT2_DIR, OUTPUT);
	pinMode(MOT2_BRK, OUTPUT);
	analogWriteFrequency(MOT2_PWM, PWM_FREQUENCY);

	pinMode(MOT3_PWM, OUTPUT);
	pinMode(MOT3_DIR, OUTPUT);
	pinMode(MOT3_BRK, OUTPUT);
	analogWriteFrequency(MOT3_PWM, PWM_FREQUENCY);
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif
}

void MotorControl::control() {
#if defined(TRIKE)
	controlTrike();
#elif defined(DIFFERENTIAL)
	controlDifferential();
#elif defined(HOLONOMIC)
	controlHolonomic();
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

}

void MotorControl::controlTrike() {

}

void MotorControl::controlDifferential() {
}

void MotorControl::controlHolonomic() {

//	if(_targetSpeed.getVx() == 0){
//		digitalWrite(2, HIGH);
//	}
//	else {
//		digitalWrite(2, LOW);
//	}


	float32_t m_data[] =
				   {0,
				    0,
				    0};

	arm_matrix_instance_f32 m = {
			.numRows = 3,
			.numCols = 1,
			.pData = m_data
	};

	arm_status status = arm_mat_mult_f32(&D, _targetSpeed, &m);

	if(status != ARM_MATH_SUCCESS) {
		Serial.print("[ERROR] controlHolonomic: matrix multiplication error : ");
		Serial.println(status);
	}


	arm_matrix_instance_f32* speed = odometry.getMotorSpeeds();	//get current speed

	//compute error
	float32_t error1 = m.pData[0] - speed->pData[0];
	float32_t error2 = m.pData[1] - speed->pData[1];
	float32_t error3 = m.pData[2] - speed->pData[2];

	//update integral error
	_intError[0] = clamp((float32_t)-INT_CLAMP, _intError[0] + error1, (float32_t)INT_CLAMP);
	_intError[1] = clamp((float32_t)-INT_CLAMP, _intError[1] + error2, (float32_t)INT_CLAMP);
	_intError[2] = clamp((float32_t)-INT_CLAMP, _intError[2] + error3, (float32_t)INT_CLAMP);

	//compute deriv error
	float32_t derivError1 = error1 - _prevError[0];
	float32_t derivError2 = error2 - _prevError[1];
	float32_t derivError3 = error3 - _prevError[2];

	//store error for future use
	_prevError[0] = error1;
	_prevError[1] = error2;
	_prevError[2] = error3;

	//compute command
	float32_t cmd1 = KP[0] * error1 + KI[0] * _intError[0] + KD[0] * derivError1;
	float32_t cmd2 = KP[1] * error2 + KI[1] * _intError[1] + KD[1] * derivError2;
	float32_t cmd3 = KP[2] * error3 + KI[2] * _intError[2] + KD[2] * derivError3;

	//clamp command between 0 and 255
	int cons1 = clamp(-PWM_MAX, (int)cmd1, PWM_MAX);
	int cons2 = clamp(-PWM_MAX, (int)cmd2, PWM_MAX);
	int cons3 = clamp(-PWM_MAX, (int)cmd3, PWM_MAX);

Serial.print(m.pData[0]);
Serial.print(";");
Serial.print(speed->pData[0]);
Serial.print(";");
Serial.print(error1);
Serial.print(";");
Serial.print(cmd1);
Serial.print(";");
Serial.println(cons1);

	setMotorCommand(cons1, MOT1_PWM, MOT1_DIR);
	setMotorCommand(cons2, MOT2_PWM, MOT2_DIR);
	setMotorCommand(cons3, MOT3_PWM, MOT3_DIR);

#ifdef SIMULATOR
	simulator.setMotorCommand(cons1, 0);
	simulator.setMotorCommand(cons2, 1);
	simulator.setMotorCommand(cons3, 2);
#endif

	prev_cons[0] = cons1;
	prev_cons[1] = cons2;
	prev_cons[2] = cons3;

}

void MotorControl::setMotorCommand(int command, int pwmPin, int dirPin) {
	digitalWrite(dirPin, sig(command));
	analogWrite(pwmPin, abs(command));
}
