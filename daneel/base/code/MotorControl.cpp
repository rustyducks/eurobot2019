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


	//set motors directions
	digitalWrite(MOT1_DIR, sig(cmd1));
	digitalWrite(MOT2_DIR, sig(cmd2));
	digitalWrite(MOT3_DIR, sig(cmd3));

#ifdef SIMULATOR
	simulator.digitalWrite(0, sig(cmd1));
	simulator.digitalWrite(1, sig(cmd2));
	simulator.digitalWrite(2, sig(cmd3));
#endif

	//clamp command between 0 and 255
	int cons1 = clamp(0, (int)abs(cmd1), 255);
	int cons2 = clamp(0, (int)abs(cmd2), 255);
	int cons3 = clamp(0, (int)abs(cmd3), 255);

	//clamp it to avoid brutal commands
//	cons1 = clamp(prev_cons[0] - MAX_CONS_DIFF, cons1, prev_cons[0] + MAX_CONS_DIFF);
//	cons2 = clamp(prev_cons[1] - MAX_CONS_DIFF, cons2, prev_cons[1] + MAX_CONS_DIFF);
//	cons3 = clamp(prev_cons[2] - MAX_CONS_DIFF, cons3, prev_cons[2] + MAX_CONS_DIFF);


Serial.print(m.pData[0]);
Serial.print(";");
Serial.print(speed->pData[0]);
Serial.print(";");
Serial.print(error1);
Serial.print(";");
Serial.print(cmd1);
Serial.print(";");
Serial.println(cons1);

	analogWrite(MOT1_PWM, cons1);
	analogWrite(MOT2_PWM, cons2);
	analogWrite(MOT3_PWM, cons3);

#ifdef SIMULATOR
	simulator.analogWrite(0, cons1);
	simulator.analogWrite(0, cons2);
	simulator.analogWrite(0, cons3);
#endif



	prev_cons[0] = cons1;
	prev_cons[1] = cons2;
	prev_cons[2] = cons3;

}
