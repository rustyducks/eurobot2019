// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"

Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));


void setup()
{
	initOdometry();
	controlTime.reset();
}

// The loop function is called in an endless loop
void loop()
{
	if(controlTime.check()) {
		odometry.update();
		motorControl.control();
	}

}
