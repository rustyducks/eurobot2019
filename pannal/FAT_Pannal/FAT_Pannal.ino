#include <Servo.h>
#include "params.h"

unsigned long sun_time, train_time, fat_time = 0;

int sens_sun = 0;
int train_state = 0; //curent state of STEP pin
int current_sun_angle = 0;

Servo sun_servo;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{

  Serial.begin(115200);
  //just to be safe
  pinMode(SERVO1, OUTPUT);
	pinMode(SERVO2, OUTPUT);
	pinMode(SERVO3, INPUT);
  
	pinMode(SUN_PIN, OUTPUT);
	pinMode(F_PIN, OUTPUT);
	pinMode(A_PIN, OUTPUT);
	pinMode(T_PIN, OUTPUT);
	pinMode(STEPPER_DIR, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_ENABLE, OUTPUT);

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

    float fake_angle = map_float(current_sun_angle, MIN_SUN_US, MAX_SUN_US, 0, PI);
    int shinning = sin(fake_angle) * 255;
    analogWrite(SUN_PIN, shinning);
    
    //Serial.println(current_sun_angle);
  }

  //------------------FAT---------------------
  //rien pour l'instant
  if(_time - fat_time > FAT_PERIOD) {
    static int fat_clock = 0;
    fat_time = _time;

    switch(fat_clock) {
      case 0:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, LOW);
        digitalWrite(T_PIN, LOW);
        Serial.println("F");
        break;
      case 1:
        digitalWrite(F_PIN, LOW);
        digitalWrite(A_PIN, HIGH);
        digitalWrite(T_PIN, LOW);
        Serial.println("A");
        break;
      case 2:
        digitalWrite(F_PIN, LOW);
        digitalWrite(A_PIN, LOW);
        digitalWrite(T_PIN, HIGH);
        Serial.println("T");
        break;
      case 3:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, LOW);
        digitalWrite(T_PIN, LOW);
        Serial.println("F");
        break;
      case 4:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, HIGH);
        digitalWrite(T_PIN, LOW);
        Serial.println("FA");
        break;
      case 5:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, HIGH);
        digitalWrite(T_PIN, HIGH);
        Serial.println("FAT");
        break;
      case 6:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, HIGH);
        digitalWrite(T_PIN, HIGH);
        Serial.println("FAT");
        break;
      case 7:
        digitalWrite(F_PIN, HIGH);
        digitalWrite(A_PIN, HIGH);
        digitalWrite(T_PIN, HIGH);
        Serial.println("FAT");
        break;
    }

    fat_clock = (fat_clock + 1) % FAT_CLOCK_PERIOD;
    
  }
 
}

