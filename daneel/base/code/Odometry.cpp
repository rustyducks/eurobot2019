/*
 * BaseHolonomy.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <Odometry.h>
#include "params.h"
#include "utilities.h"

#ifdef SIMULATOR
#include "Simulator.h"
#endif

Odometry odometry = Odometry();

volatile uint32_t v_read;   // Some variables have to be read before they can be changed.
	//Needs to be global volatile otherwise the compiler will optimize the read.


Odometry::Odometry() {
	_x = _y = _theta = 0;
	_speed = _omega = 0;
	_incr1 = _incr2 = 0;
	lastLeftCTN = COUNTER_ZERO_VALUE;
}

Odometry::~Odometry() {
}

static inline int readLeftInc() {
	noInterrupts();	    // Variables can change if FTM interrrupt occurs
	uint16_t count16 = FTM1_CNT;
	interrupts();   // Turn interrupts back on before return
	return count16/* - 32000*/;
}

void Odometry::update() {
	cli();
	int incr1 = _incr1;
	int incr2 = _incr2;
	_incr1 = _incr2 = 0;
	sei();

#ifdef SIMULATOR
	incr1 = simulator.readEnc(0);
	incr2 = simulator.readEnc(1);
#endif

	float length = ((float)(-incr1+incr2)/2.0)/INC_PER_MM;		//opposite sign on incr1 because motors are mirrored
	float angle = ((float)(incr1+incr2)/INC_PER_MM)/WHEELBASE;  //opposite sign on incr1 because motors are mirrored

	_x = _x + length*cos(_theta + angle/2.0);
	_y = _y + length*sin(_theta + angle/2.0);
	_theta = center_radians(_theta + angle);
	_speed = length / CONTROL_PERIOD;
	_omega = angle / CONTROL_PERIOD;



	noInterrupts();	    // Variables can change if FTM interrrupt occurs
	int32_t leftCTN = (int32_t)FTM1_CNT;
	interrupts();   // Turn interrupts back on before return

	int32_t delta = leftCTN - lastLeftCTN;
	lastLeftCTN = leftCTN;

	if(FTM1_SC & FTM_SC_TOF) {		//overflow occurs
		FTM1_SC=0;		//clear overflow bit
		int tofdir = FTM1_QDCTRL & FTM_QDCTRL_TOFDIR;	//determine overflow direction
		if(tofdir) {			//overflow counting up
			delta += 65535;
		}
		else {					////overflow counting down
			delta -= 65535;
		}
	}

	//do something with delta

	//Serial.print("  Test odom: ");
	//Serial.println(delta);
}

void Odometry::init() {
	reset();
}


void initOdometry() {
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);

	initLeftEncoder();

	odometry.init();
}

void ISR1() {
	odometry.isr1();
}

void ISR11() {
	odometry.isr11();
}

void ISR2() {
	odometry.isr2();
}

void ISR22() {
	odometry.isr22();
}

void Odometry::isr1() {
	if(digitalRead(MOT1_ENCB)) {
		_incr1++;
	} else {
		_incr1--;
	}
}

void Odometry::isr11() {
	if(digitalRead(MOT1_ENCA)) {
		_incr1--;
	} else {
		_incr1++;
	}
}

void Odometry::isr2() {
	if(digitalRead(MOT2_ENCB)) {
		_incr2++;
	} else {
		_incr2--;
	}
}

void Odometry::isr22() {
	if(digitalRead(MOT2_ENCA)) {
		_incr2--;
	} else {
		_incr2++;
	}
}

void Odometry::reset() {
	cli();
	_x = _y = _theta = 0;
	_speed = _omega = 0;
	_incr1 = _incr2 = 0;
	sei();
}

void Odometry::zeroLeftFTM() {
#warning "Unexepected behavior if you use it quite often (read 1 even if the robot has not moved)"
    // Turn off counter to disable interrupts and clear any
    //   overflow and compare bits

    v_read=FTM1_FMS;	// Requires reading before WPDIS can be set

    noInterrupts();
    FTM1_MODE=0x05;	// Write Protect Disable with FTMEN set
    FTM1_QDCTRL=0xE0;	// Disable QUADEN, filter still enabled
                        //   Turns off counter so no interrupts
    interrupts();

    FTM1_CNTIN=COUNTER_ZERO_VALUE;	//set initial value to 32000 (the middle to not bother with overflows)
    FTM1_CNT=0;		// Updates counter with CNTIN
    FTM1_CNTIN=0;	//set minimum value to 0

    FTM1_C0SC=0x10;	// Disable Channel compare int, clear compare bit
			//   Leaves Mode bit for compare set

    // Enable counter again
    FTM1_QDCTRL=0xE1;	// QUADEN
    FTM1_FMS=0x40;  	// Write Protect, WPDIS=1
}

void initLeftEncoder() {

	PORTA_PCR12 = 0x00000712;   //Alt7-QD_FTM1,FilterEnable,Pulldown
	PORTA_PCR13 = 0x00000712;   //Alt7-QD_FTM1,FilterEnable,Pulldown

    //Set FTMEN to be able to write registers
	FTM1_MODE=0x04;	    // Write protect disable - reset value
	FTM1_MODE=0x05;	    // Set FTM Enable

    // Set registers written in pins_teensy.c back to default
    FTM1_CNT = 0;
    FTM1_MOD = 0;
    FTM1_C0SC =0;
    FTM1_C1SC =0;
    FTM1_SC = 0;


    // Set registers to count quadrature
    FTM1_FILTER=0x22;	// 2x4 clock filters on both channels
    FTM1_MOD=0xFFFF;	// Maximum value of counter
    FTM1_CNTIN=COUNTER_ZERO_VALUE;	//set initial value to 32000 (the middle to not bother with overflows)
    FTM1_CNT=0;		// Updates counter with CNTIN
    FTM1_CNTIN=0;	//set minimum value to 0

    // Set Registers for output compare mode
    FTM1_COMBINE=0;	    // Reset value, make sure
    FTM1_C0SC=0x10;	    // Bit 4 Channel Mode

    FTM1_QDCTRL=0b11100001;	    // Quadrature control		0xE1
    //        Filter enabled, phase A inverted (positive count down) ,QUADEN set

    // Write Protect Enable
    FTM1_FMS=0x40;		// Write Protect, WPDIS=1

    //NVIC_ENABLE_IRQ(IRQ_FTM1);

}
