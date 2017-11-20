/*
 * params.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef PARAMS_H_
#define PARAMS_H_
#include "Arduino.h"

const int MOT1_PWM = 21;
const int MOT1_DIR = 23;
const int MOT1_BRK = 22;
const int MOT1_CURRENT = A6;
const int MOT1_ENCA = 17;
const int MOT1_ENCB = 16;

const int MOT2_PWM = 35;
const int MOT2_DIR = 36;
const int MOT2_BRK = 39;
const int MOT2_CURRENT = A22;
const int MOT2_ENCA = 14;
const int MOT2_ENCB = 15;

const int MOT3_PWM = 30;
const int MOT3_DIR = 29;
const int MOT3_BRK = 28;
const int MOT3_CURRENT = A21;
const int MOT3_ENCA = 27;
const int MOT3_ENCB = 26;

const float PWM_FREQUENCY = 58593.75;
const int PWM_RESOLUTION = 10;
const int PWM_MAX = pow(2, PWM_RESOLUTION) - 1;

const float CONTROL_PERIOD = 0.02;

const float RAD_TO_DYNA = 16.615776058793873;

#define TRIKE
//#define HOLONOMIC


//Configurations for each robot type
#if defined(TRIKE)
const float STEER_DYNA_ID = 2;
const int  DYNA_ZERO = 512;		//512 is the middle angle for dynamixels
const float WHEELBASE = 200.0;
const float WHEEL_PERIMETER = 239.389;
const float INCREMENTS_PER_ROTATION = 1600.0; //3200 if using both raising and falling edge of both A and B channel
const float MINIMUM_ANGLE = 10-2;

const float KP = 1;
const float KI = 0;
const float KD = 0;


#elif defined(DIFFERENTIAL)
	//plop
#elif defined(HOLONOMIC)

const int ROBOT_RADIUS = 100;

#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

#endif /* PARAMS_H_ */
