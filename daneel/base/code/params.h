/*
 * params.h
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#ifndef PARAMS_H_
#define PARAMS_H_
#include "Arduino.h"
#include "arm_math.h"

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

const float PWM_FREQUENCY = 915.527;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = pow(2, PWM_RESOLUTION) - 5;

const float CONTROL_PERIOD = 0.05;
const float POS_REPORT_PERIOD = 0.2;

#define HOLONOMIC


//Configurations for each robot type
#if defined(TRIKE)
const int STEER_DYNA_ID = 2;
const int  DYNA_ZERO = 605;		//512 is the middle angle for dynamixels
const float WHEELBASE = 200.0;
const float WHEEL_PERIMETER = 239.389;
const float INCREMENTS_PER_ROTATION = 1600.0; //3200 if using both raising and falling edge of both A and B channel
const float MINIMUM_ANGLE = 10-2;

const float KP = 0.4;
const float KI = 0.1;
const float KD = 0.0;


#elif defined(DIFFERENTIAL)
	//plop
#elif defined(HOLONOMIC)

const float32_t ROBOT_RADIUS = 170.0;

const float INC_PER_MM = 8.488263631567753;

const int MAX_CONS_DIFF = 50;

extern arm_matrix_instance_f32 Dplus;
extern arm_matrix_instance_f32 D;

#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

#endif /* PARAMS_H_ */
