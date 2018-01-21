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

//#define SIMULATOR

const int LED_PIN = 13;

/* BEGIN -----------------Motors & Odometry-------------------------- */

const int MOT3_PWM = 21;
const int MOT3_DIR = 23;
const int MOT3_ENCA = 17;
const int MOT3_ENCB = 16;

const int MOT1_PWM = 35;
const int MOT1_DIR = 36;
const int MOT1_ENCA = 14;
const int MOT1_ENCB = 15;

const int MOT2_PWM = 30;
const int MOT2_DIR = 29;
const int MOT2_ENCA = 27;
const int MOT2_ENCB = 26;

const float PWM_FREQUENCY = 915.527;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = pow(2, PWM_RESOLUTION) - 5;

const float32_t INC_PER_MM = 7.554804140138556;

#define HOLONOMIC
const float CONTROL_PERIOD = 0.05;

const float32_t ROBOT_RADIUS = 170.0;

/* END ------------------- Motors & Odometry --------------------------*/

/* BEGIN ----------------------- HMI ----------------------------------*/
const int LED_RED = 0;
const int LED_GREEN = 0;
const int LED_BLUE = 0;
const int CORD = 0;
const int BUTTON1 = 0;
const int BUTTON2 = 0;
/* END ------------------------- HMI ----------------------------------*/

/* BEGIN ----------------------- IOs ----------------------------------*/
const HardwareSerial RASPI_COMMUNICATION_SERIAL = Serial1;
const int RASPI_COMMUNICATION_BAUDRATE = 115200;
const int DYNAMIXEL_CONTROL = 24;
const float IO_REPORT_PERIOD = 0.5;
/* END ------------------------- IOs ----------------------------------*/


const float POS_REPORT_PERIOD = 0.2;

#define HOLONOMIC

#if defined(HOLONOMIC)

extern arm_matrix_instance_f32 Dplus;
extern arm_matrix_instance_f32 D;

#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

#endif /* PARAMS_H_ */
