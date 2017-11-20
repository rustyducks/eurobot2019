/*
 * Move3D.cpp
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#include <Move3D.h>

Move3D::Move3D() {
	_x = _y = _theta = 0;
}

Move3D::~Move3D() {
	// TODO Auto-generated destructor stub
}

Move3D::Move3D(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
	_id = 0;
}
