// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"
#include "utilities.h"
#include "ExtNavigation.h"
//#include "DynamixelSerial5.h"



Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));

Metro testTime = Metro(4000);

float32_t testCommands[][3] = {
		{0, 0, W_to_RW(600)},
		{0, 0, W_to_RW(0)},
		{0, 0, W_to_RW(-600)},
		{0, 0, W_to_RW(0)}
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
}

// The loop function is called in an endless loop
void loop()
{
	if(controlTime.check()) {
		odometry.update();
		motorControl.control();
		extNavigation.update();
	}

	if(testTime.check()) {
		//motorControl.setTargetSpeed(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
		extNavigation.setTableSpeedCons(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
		i = (i+1) % 4;
	}

//	if(Serial.available()) {
//		int cmd = Serial.read();
//		analogWrite(MOT1_PWM, cmd);
//		Serial.flush();
//	}

}
