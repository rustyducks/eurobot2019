/*
 * ExtNavigation.cpp
 *
 *  Created on: 12 déc. 2017
 *      Author: fabien
 */

#include <ExtNavigation.h>
#include "Arduino.h"
#include "Odometry.h"
#include "utilities.h"
#include "MotorControl.h"

ExtNavigation extNavigation = ExtNavigation();

ExtNavigation::ExtNavigation() {
	table_speed_cons = makeSpeed(0,0,0);
	rotation_matrix = makeMatrix(3,3);
}

ExtNavigation::~ExtNavigation() {
	// TODO Auto-generated destructor stub
}

void ExtNavigation::update() {
	computeRotationMatrix();	//compute the new rotation matrix associated with the new theta

	//instanciate speed (local variable)
	float32_t data[] = {0,0,0};
	arm_matrix_instance_f32 robot_speed;
	robot_speed.numCols = 1;
	robot_speed.numRows = 3;
	robot_speed.pData = data;

	//compute speed in the robot reference frame
	arm_status status = arm_mat_mult_f32(rotation_matrix, table_speed_cons, &robot_speed);
	if(status != ARM_MATH_SUCCESS) {
		Serial.print("[ERROR] ExtNavigation::update(): matrix multiplication error : ");
		Serial.println(status);
	}

	//set target speed to the motor controller (robot reference frame)
	motorControl.setTargetSpeed(&robot_speed);
}

void ExtNavigation::computeRotationMatrix() {
	float32_t theta = odometry.getTheta();
	float32_t cos_t = cos(theta);
	float32_t sin_t = sin(theta);

	float32_t rot_mat_data[] = {		//TODO : check if it is the correct matrix : it might be the transpose one
			cos_t, sin_t, 0,
			-sin_t,  cos_t, 0,
		    	0,      0, 1
	};
	//just to not write 9 lines... But it should works !
	memcpy(rotation_matrix->pData, rot_mat_data, 9*sizeof(float32_t));
}

void ExtNavigation::reset() {
	zeroMatrix(rotation_matrix);
	zeroMatrix(table_speed_cons);
}
