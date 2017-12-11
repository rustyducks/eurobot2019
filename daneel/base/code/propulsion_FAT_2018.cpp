// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"
//#include "DynamixelSerial5.h"

Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));

Metro testTime = Metro(4000);
Speed3D testCommands[] = {
		Speed3D(100, 0, 0),
		Speed3D(0, 100, 0),
		Speed3D(0, 0, 0.1),

		Speed3D(200, 0, 0),
		Speed3D(250, 0, 0),
		Speed3D(200, 0, 0),
		Speed3D(100, 0, 0),
		Speed3D(50, 0, 0),
		Speed3D(0.2, 0, 0.2),
		Speed3D(-50.2, 0, 0),
		Speed3D(-100, 0, -0.2),
		Speed3D(-150, 0, 0),
		Speed3D(-200, 0, 0),
		Speed3D(-250, 0, 0),
		Speed3D(-200, 0, 0),
		Speed3D(-100, 0, 0),
		Speed3D(-50, 0, 0),
		Speed3D(0, 0, 0),
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
		//int current = analogRead(MOT1_CURRENT);
		//Serial.println(current);
	}

	if(testTime.check()) {
		motorControl.setTargetSpeed(testCommands[i]);
		i = (i+1) % 3;
	}

//	if(Serial.available()) {
//		int cmd = Serial.read();
//		analogWrite(MOT1_PWM, cmd);
//		Serial.flush();
//	}

}
