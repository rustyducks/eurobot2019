/*
 * InputOuputs.cpp
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#include <InputOuputs.h>
#include "DynamixelSerial5.h"
#include "params.h"

InputOuputs::InputOuputs() {
	// TODO Auto-generated constructor stub

}

InputOuputs::~InputOuputs() {
	// TODO Auto-generated destructor stub
}

void InputOuputs::init() {
	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
}
