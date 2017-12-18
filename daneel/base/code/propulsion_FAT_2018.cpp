// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include <Arduino.h>
#include "communication/Communication.h"

using namespace fat;
Communication comm(Serial1, 115200);  // Initialisation, nanani global nanana, mais c'est juste pour la démo
bool blink;
unsigned long blinkTime;

void testHMICallback(const Communication::HMICommand msg);  // Forward declaration
void testActuatorCallback(const Communication::ActuatorCommand msg);
void testSpeedCallback(const Communication::SpeedCommand msg);

void setup()
{
	Serial.begin(115200);
	blinkTime = millis();
	blink = false;
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	digitalWrite(13, blink);
	comm.registerHMICommandCallback(testHMICallback);
	comm.registerActuatorCommandCallback(testActuatorCallback);
	comm.registerSpeedCommandCallback(testSpeedCallback);
	Serial.print("test");
}


void loop()
{
	if(millis() - blinkTime >= 1500){
		blink ^= 1;
		digitalWrite(13, blink);
		blinkTime = millis();
		comm.sendOdometryReport(500, 150, 0);
	}
	comm.checkMessages();
//Add your repeated code here
}

void testHMICallback(const Communication::HMICommand msg){
	Serial.print("Force rouge : ");
	Serial.print(msg.redLedCommand);
	Serial.print("\tForce verte : ");
	Serial.print(msg.greenLedCommand);
	Serial.print("\tForce bleue : ");
	Serial.println(msg.blueLedCommand);
}

void testActuatorCallback(const Communication::ActuatorCommand msg){
	Serial.print("Actuator Id : ");
	Serial.print(msg.actuatorId);
	Serial.print("\tActuator command : ");
	Serial.println(msg.actuatorCommand);
}

void testSpeedCallback(const Communication::SpeedCommand msg){
	Serial.print("Speed : vx : ");
	Serial.print(msg.vx);
	Serial.print("\tvy : ");
	Serial.print(msg.vy);
	Serial.print("\tvtheta : ");
	Serial.println(msg.vtheta);
}
