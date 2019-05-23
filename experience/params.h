/*
 * params.h
 *
 *  Created on: 23 févr. 2018
 *      Author: fabien
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "Arduino.h"

#define SERVO1 A0
#define SERVO2 A1
#define SERVO3 A2
#define MOSFET_UL 5
#define MOSFET_BL 6
#define MOSFET_UR 11
#define MOSFET_BR 10

#define RED MOSFET_UR
#define BLUE MOSFET_UL
#define GREEN MOSFET_BL

#define STEPPER_DIR 8
#define STEPPER_STEP 9
#define STEPPER_ENABLE 7

#define ELECTRON_PERIOD 3

#endif /* PARAMS_H_ */
