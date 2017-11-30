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
#include <DynamixelSerial5.h>

#define sig(x) (x) > 0 ? 1 : 0

MotorControl motorControl = MotorControl();

MotorControl::MotorControl() {
	_intSpeedError = Speed3D();
	_intHeadingError = 0;
	_lastMotorCmd = 0;

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
	Speed3D error = _targetSpeed - odometry.getSpeed();
	_intSpeedError = _intSpeedError + error;	//TODO operator +=
	Speed3D derivError = _previousSpeedError - error;

	Speed3D cmd = error*(float)0.3;

	_targetSpeed.print("target: ");
	Serial.print("\t\t");
	(odometry.getSpeed()).print("speed : ");
	Serial.print("\t\t");
	error.print("error: ");
	Serial.print("\t\t");
	cmd.print("cmd: ");
	Serial.println("");

	float32_t v_data[] =
		       {cmd.getVx(),
			    cmd.getVy(),
			    cmd.getOmega()};

	arm_matrix_instance_f32 v = {
			.numRows = 3,
			.numCols = 1,
			.pData = v_data
	};


	float32_t m_data[] =
				   {1,
				    1,
				    1};

	arm_matrix_instance_f32 m = {
			.numRows = 3,
			.numCols = 1,
			.pData = m_data
	};

	arm_status status = arm_mat_mult_f32(&D, &v, &m);

	if(status != ARM_MATH_SUCCESS) {
		Serial.print("[ERROR] controlHolonomic: matrix multiplication error : ");
		Serial.println(status);
	}
	//int dir = 1;
	//if(_targetSpeed.getVx() < 0) { dir = 0;}
	digitalWrite(MOT1_DIR, sig(m.pData[0]));
	digitalWrite(MOT2_DIR, sig(m.pData[1]));
	digitalWrite(MOT3_DIR, sig(m.pData[2]));

	int cons1 = clamp(0, (int)abs(m.pData[0]), 255);
	int cons2 = clamp(0, (int)abs(m.pData[1]), 255);
	int cons3 = clamp(0, (int)abs(m.pData[2]), 255);
/*
	Serial.print(status);
	Serial.print("\t");
	Serial.print(m.pData[0]);
	Serial.print("\t");
	Serial.print(m.pData[1]);
	Serial.print("\t");
	Serial.println(m.pData[2]);
*/
	analogWrite(MOT1_PWM, cons1);
	analogWrite(MOT2_PWM, cons2);
	analogWrite(MOT3_PWM, cons3);

}
