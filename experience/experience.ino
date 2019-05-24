#include <ServoTimer2.h>
#include <VirtualWire.h>
//#include <Servo.h>
#include "params.h"
// ACM0

bool get_signal();

unsigned long electron_time = 0;
unsigned long led_time = 0;
unsigned long dog_time = 0;

int nb_step = 0; //curent state of STEP pin

int led_state = 0;

//int servo_position[] = {45, 45, 70, 60, 70};
int servo_position[] = {1250, 1250, 1390, 1330, 1390};
int servo_i;

int blink_led = 13; //DEBUG

ServoTimer2 dog_servo;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  vw_set_rx_pin(A5);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);   // Bits per sec

  vw_rx_start();       // Start the receiver PLL running
  pinMode(blink_led, OUTPUT);   //DEBUG
  Serial.begin(9600);
  //just to be safe
  //pinMode(SERVO1, OUTPUT);
	pinMode(SERVO2, OUTPUT);
	//pinMode(SERVO3, INPUT);
  dog_servo.attach(SERVO2);
  dog_servo.write(45);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  
	pinMode(STEPPER_DIR, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_ENABLE, OUTPUT);

  digitalWrite(STEPPER_DIR, HIGH);
  digitalWrite(STEPPER_ENABLE, LOW);
  digitalWrite(STEPPER_STEP, LOW);


  while(!get_signal());

  //delay(5000);
}


void loop()
{
  
	unsigned long _time = millis();

  //-------------------- train -------------------------
	if((_time - electron_time) > ELECTRON_PERIOD && nb_step < 3800) {
    electron_time = _time;
		nb_step++;
		digitalWrite(STEPPER_STEP, nb_step%2);
	}

 if(_time - led_time > 1000) {
    led_time = _time;
    led_state = (led_state+1)%6;
    if(led_state == 0) {
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
    }
    if(led_state == 1 || led_state == 2) {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(BLUE, LOW);
    }
    if(led_state == 3 || led_state == 4 || led_state == 5) {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
    }
 }

 if(_time - dog_time > 500) {
    digitalWrite(blink_led, servo_i%2);  
    dog_time = _time;
    servo_i = (servo_i + 1) % 5;
    dog_servo.write(servo_position[servo_i]);
  
 }
 
}


bool get_signal() {
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;


  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    int i;

    digitalWrite(13, true); // Flash a light to show received good message
    // Message with a good checksum received, dump it.
    Serial.print("Got: ");
  
    for (i = 0; i < buflen; i++)
    {
        char b = buf[i];
        Serial.print(b);
        //Serial.print(" ");
    }
    Serial.println("");

    if(strcmp(buf, "coincoin") == 0) {
      return true;
    }
    
    digitalWrite(13, false);
  }
  return false;
}
