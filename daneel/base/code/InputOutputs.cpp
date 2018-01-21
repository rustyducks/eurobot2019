/*
 * InputOuputs.cpp
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#include <InputOutputs.h>
#include "DynamixelSerial5.h"
#include "params.h"
#include "communication/Communication.h"

using namespace fat;

InputOutputs inputOutputs = InputOutputs();

void ioOnNewCordState();
void ioOnNewButton1State();
void ioOnNewButton2State();

InputOutputs::InputOutputs(): button1Pressed(false), button2Pressed(false), cordIn(false),
		redLEDOn(false), greenLEDOn(false), blueLEDOn(false) {
	// TODO Auto-generated constructor stub

}

InputOutputs::~InputOutputs() {
	// TODO Auto-generated destructor stub
}

void InputOutputs::init() {
	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	pinMode(CORD, INPUT_PULLUP);
	pinMode(BUTTON1, INPUT_PULLUP);
	pinMode(BUTTON2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CORD), ioOnNewCordState, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1), ioOnNewButton1State, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON2), ioOnNewButton2State, CHANGE);

	HMISetLedColor(0, 0, 0);
}

void InputOutputs::_onNewCordState(){
	cordIn = digitalRead(CORD);
	communication.sendIHMState(cordIn, button1Pressed, button2Pressed, redLEDOn, greenLEDOn, blueLEDOn);
}

bool InputOutputs::HMIGetButton1State() {
	button1Pressed = digitalRead(BUTTON1);
	return button1Pressed;
}

bool InputOutputs::HMIGetButton2State() {
	button2Pressed = digitalRead(BUTTON2);
	return button2Pressed;
}

bool InputOutputs::HMIGetCordState() {
	cordIn = digitalRead(CORD);
	return cordIn;
}

void InputOutputs::_onNewButtonState(int button){
	if (button == 1){
		button1Pressed = digitalRead(BUTTON1);
	}else if(button == 2){
		button2Pressed = digitalRead(BUTTON2);
	}
	communication.sendIHMState(cordIn, button1Pressed, button2Pressed, redLEDOn, greenLEDOn, blueLEDOn);
}

void InputOutputs::HMISetLedColor(int red, int green, int blue){
	redLEDOn = red;
	greenLEDOn = green;
	blueLEDOn = blue;
	digitalWrite(LED_RED, redLEDOn ? HIGH : LOW);
	digitalWrite(LED_GREEN, greenLEDOn ? HIGH : LOW);
	digitalWrite(LED_BLUE, blueLEDOn ? HIGH : LOW);
	HMISendState();
}

void InputOutputs::HMISendState(){
	cordIn = digitalRead(CORD);
	button1Pressed = digitalRead(BUTTON1);
	button2Pressed = digitalRead(BUTTON2);
	communication.sendIHMState(cordIn, button1Pressed, button2Pressed, redLEDOn, greenLEDOn, blueLEDOn);
}



void ioOnNewCordState(){
	inputOutputs._onNewCordState();
}

void ioOnNewButton1State(){
	inputOutputs._onNewButtonState(1);
}

void ioOnNewButton2State(){
	inputOutputs._onNewButtonState(2);
}
