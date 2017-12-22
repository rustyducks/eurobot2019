// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include <Arduino.h>
#include "communication/Communication.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"
#include "utilities.h"
#include "ExtNavigation.h"
//#include "DynamixelSerial5.h"
#ifdef SIMULATOR
#include "Simulator.h"
#endif

using namespace fat;
Communication comm(Serial1, 115200);  // Initialisation, nanani global nanana, mais c'est juste pour la démo
bool blink;
unsigned long blinkTime;

void testHMICallback(const Communication::HMICommand msg);  // Forward declaration
void testActuatorCallback(const Communication::ActuatorCommand msg);
void setNewTableSpeedCallback(const Communication::SpeedCommand msg);


Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));
Metro posReportTme = Metro((unsigned long)(POS_REPORT_PERIOD*1000));

Metro testTime = Metro(4000);

float32_t testCommands[][3] = {
		{0, 0, 1},
		{0, 0, 0},
		{0, 0, -1},
		{0, 0, 0}
};
int i = 0;

void setup()
{
	//pinMode(25, OUTPUT);
	//digitalWrite(25, HIGH);
	initOdometry();
	motorControl.init();
	Serial.begin(115200);
	while(!Serial.available());
	Serial.println("Start");
	Serial.flush();
	testTime.reset();
	controlTime.reset();
	
	blinkTime = millis();
	blink = false;
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, blink);
	comm.registerHMICommandCallback(testHMICallback);
	comm.registerActuatorCommandCallback(testActuatorCallback);
	comm.registerSpeedCommandCallback(setNewTableSpeedCallback);
}


void loop()
{
	if(millis() - blinkTime >= 1500){
		blink ^= 1;
		digitalWrite(LED_PIN, blink);
		blinkTime = millis();
	}
	comm.checkMessages();

	if(controlTime.check()) {
#ifdef SIMULATOR
		simulator.update();
#endif
		odometry.update();
		motorControl.control();
		extNavigation.update();
	}

	if(posReportTme.check()) {
		//comm.sendMove(odometry.getMoveDelta())
		odometry.resetMoveDelta();
	}

	/*if(testTime.check()) {
		//motorControl.setTargetSpeed(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
		extNavigation.setTableSpeedCons(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
		i = (i+1) % 4;
	}*/

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

void setNewTableSpeedCallback(const Communication::SpeedCommand msg){
	extNavigation.setTableSpeedCons(msg.vx, msg.vy, msg.vtheta);
}
