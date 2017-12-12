/*
 * utilities.cpp
 *
 *  Created on: 11 déc. 2017
 *      Author: fabien
 */


#include "utilities.h"
#include "arm_math.h"
#include "Arduino.h"

arm_matrix_instance_f32* makeSpeed(float32_t vx, float32_t vy, float32_t w) {

	arm_matrix_instance_f32* speed = NULL;
	speed = (arm_matrix_instance_f32*) malloc(sizeof(arm_matrix_instance_f32));
	float32_t* speed_data = NULL;
	speed_data = (float32_t*) malloc(3*sizeof(float32_t));

	if(speed == NULL || speed_data == NULL) {
		Serial.println("[ERROR] makeSpeed: malloc failed");
		return NULL;
	}

	speed->numCols = 1;
	speed->numRows = 3;
	speed->pData = speed_data;

	return speed;
}

void freeSpeed(arm_matrix_instance_f32* speed) {
	free(speed->pData);
	free(speed);
}
