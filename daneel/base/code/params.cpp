/*
 * params.c
 *
 *  Created on: 27 nov. 2017
 *      Author: fabien
 */

#include "arm_math.h"
#include "params.h"


/*
 *  |v1|   |-sin(O1)  cos(O1)  1|   |vx|
 *  |v2| = |-sin(O2)  cos(O2)  1| . |vy|
 *  |v3|   |-sin(O3)  cos(O3)  1|   |Rw|
 *
 *    m  =           D            .   v
 */


//Euclidean speeds into motor speeds: m = Dv

float32_t D_data[] =
       { 0.0      , 1.0 , 1.0,
		-0.8660254, -0.5, 1.0,
		 0.8660254, -0.5, 1.0};

arm_matrix_instance_f32 D = {
		.numRows = 3,
		.numCols = 3,
		.pData = D_data
};


//motor speeds into Euclidean speeds: v = D+ m
float32_t Dplus_data[] = {2.56395025e-16,  -5.77350269e-01,   5.77350269e-01,
		                  6.66666667e-01,  -3.33333333e-01,  -3.33333333e-01,
		                  3.33333333e-01,   3.33333333e-01,   3.33333333e-01};

arm_matrix_instance_f32 Dplus = {
		.numRows = 3,
		.numCols = 3,
		.pData = Dplus_data
};
