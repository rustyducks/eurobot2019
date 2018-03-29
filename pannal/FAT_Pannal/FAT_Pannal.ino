#include <Servo.h>
#include "params.h"
int test = 0;
unsigned long test_time = 0;

unsigned long sun_time, train_time = 0;

int sens_sun = 0;
int train_state = 0; //curent state of STEP pin
int current_sun_angle = 0;

Servo sun_servo;

void setup()
{

  Serial.begin(115200);
  //just to be safe
  pinMode(SERVO1, OUTPUT);
	pinMode(SERVO2, OUTPUT);
	pinMode(SERVO3, INPUT);
  
	pinMode(MOSFET_UL, OUTPUT);
	pinMode(MOSFET_BL, OUTPUT);
	pinMode(MOSFET_UR, OUTPUT);
	pinMode(MOSFET_BR, OUTPUT);
	pinMode(STEPPER_DIR, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_ENABLE, OUTPUT);

  digitalWrite(MOSFET_UL, LOW);
  digitalWrite(MOSFET_BL, LOW);
  digitalWrite(MOSFET_UR, LOW);
  digitalWrite(MOSFET_BR, LOW);
  digitalWrite(STEPPER_DIR, HIGH);
  digitalWrite(STEPPER_ENABLE, LOW);
  digitalWrite(STEPPER_STEP, LOW);

    sun_servo.attach(SERVO1);
    current_sun_angle = MAX_SUN_US;
    sun_servo.writeMicroseconds(current_sun_angle);
}


void loop()
{
	unsigned long _time = millis();

  //-------------------- train -------------------------
	if((_time - train_time) > TRAIN_PERIOD) {
    train_time = _time;
		train_state = !train_state;
		digitalWrite(STEPPER_STEP, train_state);
	}

  //-------------------- sun --------------------------
  if((_time - sun_time) > SUN_PERIO) {
    sun_time = _time;
    if(sens_sun) {
      current_sun_angle = min(current_sun_angle + SUN_INC, MAX_SUN_US);
      if(current_sun_angle == MAX_SUN_US) {
        sens_sun = 0;
      }
    }
    else {
      current_sun_angle = max(current_sun_angle - SUN_INC, MIN_SUN_US);
      if(current_sun_angle == MIN_SUN_US) {
        sens_sun = 1;
      }
    }
    sun_servo.writeMicroseconds(current_sun_angle);
    Serial.println(current_sun_angle);
  }

  //------------------FAT---------------------
  //rien pour l'instant
  if(_time - test_time > 500) {
    test_time = _time;
    test = !test;
    digitalWrite(MOSFET_BR, test);
  }
 
}

