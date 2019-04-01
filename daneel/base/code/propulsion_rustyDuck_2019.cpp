// Do not remove the include below
#include "propulsion_rustyDuck_2019.h"
#include <Arduino.h>
#include "communication/Communication.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"
#include "utilities.h"
#include "ExtNavigation.h"
#include "InputOutputs.h"

#ifdef SIMULATOR
#include "Simulator.h"
#endif

using namespace fat;
bool blink;
unsigned long blinkTime;

void handleLEDCallback(const Communication::HMICommand msg);
void testHMICallback(const Communication::HMICommand msg);  // Forward declaration
void testActuatorCallback(const Communication::ActuatorCommand msg);
void setNewSpeedCallback(const Communication::SpeedCommand msg);
void handleActuatorCallback(const Communication::ActuatorCommand msg);
void resetCallback();
void handleRepositionningCallback(const Communication::Repositionning msg);
void handleSensorCommandCallback(const Communication::SensorCommand cmd);


Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));
Metro posReportTme = Metro((unsigned long)(POS_REPORT_PERIOD*1000));
Metro IOsTime = Metro((unsigned long)(IO_REPORT_PERIOD*1000));
Metro BallDetectorTime = Metro((unsigned long)(100));

//Metro testTime = Metro(4000);

void setup()
{
	//pinMode(25, OUTPUT);
	//digitalWrite(25, HIGH);
	initOdometry();
	motorControl.init();
	inputOutputs.init();
	communication.init();
	Serial.begin(115200);
	//while(!Serial);
	Serial.println("Start");
	Serial.flush();
//	testTime.reset();
	controlTime.reset();
	
	blinkTime = millis();
	blink = false;
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, blink);
	communication.registerHMICommandCallback(testHMICallback);
	communication.registerHMICommandCallback(handleLEDCallback);
	communication.registerActuatorCommandCallback(testActuatorCallback);
	communication.registerActuatorCommandCallback(handleActuatorCallback);
	communication.registerSpeedCommandCallback(setNewSpeedCallback);
	communication.registerResetCallback(resetCallback);
	communication.registerRepositionningCallback(handleRepositionningCallback);
	communication.registerSensorCommandCallback(handleSensorCommandCallback);
}


void loop()
{
	if(millis() - blinkTime >= 1500){
		blink ^= 1;
		digitalWrite(LED_PIN, blink);
		blinkTime = millis();
	}
	communication.checkMessages();


	if(controlTime.check()) {
#ifdef SIMULATOR
		simulator.update();
#endif
		odometry.update();
		odometry.periodic_position();
		extNavigation.update();
		motorControl.control();

//		Serial.print("x: ");
//		Serial.print(odometry.get_pos_x());
//		Serial.print("\ty: ");
//		Serial.print(odometry.get_pos_y());
//		Serial.print("\ttheta: ");
//		Serial.print(odometry.get_pos_theta());
//		Serial.print("\tspeed: ");
//		Serial.print(odometry.get_speed());
//		Serial.print("\tomega ");
//		Serial.println(odometry.get_omega());

	}

	if(posReportTme.check()) {
		communication.sendOdometryPosition(odometry.get_pos_x(), odometry.get_pos_y(),
				odometry.get_pos_theta());
		communication.sendSpeedReport(odometry.get_speed(), 0.0, odometry.get_omega());
	}

	if(IOsTime.check()) {
		inputOutputs.run();
	}

//	if(testTime.check()) {
//	}

}

void handleLEDCallback(const Communication::HMICommand msg){
	inputOutputs.HMISetLedColor(msg.redLedCommand, msg.greenLedCommand, msg.blueLedCommand);
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

void handleActuatorCallback(const Communication::ActuatorCommand msg){
	inputOutputs.handleActuatorMessage(msg.actuatorId, msg.actuatorCommand);
	//Dynamixel.ledStatus(msg.actuatorId, msg.actuatorCommand);
}

void setNewSpeedCallback(const Communication::SpeedCommand msg){
	fat::communication.setTimeLastSpeedMessage(millis());
	extNavigation.setSpeedCons(msg.vx, msg.vtheta);
}

void resetCallback(){
	motorControl.reset();
	odometry.reset();
	extNavigation.reset();
#ifdef SIMULATOR
	simulator.reset();
#endif
	communication.reset();
	inputOutputs.reset();
}

void handleRepositionningCallback(const Communication::Repositionning msg){
	odometry.set_pos(msg.x, msg.y, msg.theta);
}

void handleSensorCommandCallback(Communication::SensorCommand cmd){
	inputOutputs.handleSensorCommand(cmd.sensorId, cmd.sensorCommand);
}
