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
	arm_matrix_instance_f32* speed = makeMatrix(3, 1);
	speed->pData[0] = vx;
	speed->pData[0] = vy;
	speed->pData[0] = w;

	return speed;
}

void freeSpeed(arm_matrix_instance_f32* speed) {
	free(speed->pData);
	free(speed);
}

arm_matrix_instance_f32* makeMatrix(int nRows, int nCols) {
	arm_matrix_instance_f32* matrix = NULL;
	matrix = (arm_matrix_instance_f32*) malloc(sizeof(arm_matrix_instance_f32));

	float32_t* data = NULL;
	data = (float32_t*) malloc(nRows*nCols*sizeof(float32_t));

	if(matrix == NULL || data == NULL) {
		Serial.println("[ERROR] makeMatrix: malloc failed");
		return NULL;
	}

	matrix->numCols = nCols;
	matrix->numRows = nRows;
	matrix->pData = data;

	return matrix;
}

arm_matrix_instance_f32* makeMove(float32_t dx, float32_t dy, float32_t dTheta) {
	arm_matrix_instance_f32* move = makeMatrix(3, 1);
	move->pData[0] = dx;
	move->pData[0] = dy;
	move->pData[0] = dTheta;

	return move;
}
