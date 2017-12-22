/*
 * ExtNavigation.h
 *
 *  Created on: 12 déc. 2017
 *      Author: fabien
 */

#ifndef EXTNAVIGATION_H_
#define EXTNAVIGATION_H_

#include "arm_math.h"
#include "utilities.h"

class ExtNavigation {
public:
	ExtNavigation();
	virtual ~ExtNavigation();

	/**
	 * Set the new commands to the motor controller according to the current theta and
	 * speed consigne (table reference frame)
	 */
	void update();

	/**
	 * Compute the new rotation matrix, according to the current theta of the robot.
	 */
	void computeRotationMatrix();

	/**
	 * \brief Set the speed consigne by memcpy of existing arm_matrix_instance_f32.
	 */
	void setTableSpeedCons(arm_matrix_instance_f32* tableSpeedCons) {
		memcpy(table_speed_cons->pData, tableSpeedCons->pData, 3*sizeof(float32_t));
	}

	/**
	 * \brief Set the speed consigne from its 3 composantes.
	 * \param vx  speed along the X axis in mm/s
	 * \param vy  speed along the Y axis in mm/s
	 * \param w   rotation speed in rd/s
	 */
	void setTableSpeedCons(float32_t vx, float32_t vy, float32_t w) {
		table_speed_cons->pData[0] = vx;
		table_speed_cons->pData[1] = vy;
		table_speed_cons->pData[2] =  W_to_RW(w);
	}

	void reset();

private:
	/**
	 * \brief The 3x3 matrix rotating a vector from the table reference frame to the
	 * robot reference frame
	 *
	 * |cosO  -sinO  0| |vxt   |vxr
	 * |sinO   cosO  0|*|vyt = |vyr  or may be the transpose of this one
	 * |0      0     1| |w     |w
	 */
	arm_matrix_instance_f32* rotation_matrix;

	/**
	 * The speed consigne (vx, vy, Rw), *in the table reference frame*.
	 * vx and vy are in mm/s
	 * R.w is in mm.rd/s
	 */
	arm_matrix_instance_f32* table_speed_cons;


};

extern ExtNavigation extNavigation;

#endif /* EXTNAVIGATION_H_ */
