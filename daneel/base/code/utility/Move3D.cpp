/*
 * Move3D.cpp
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#include <Move3D.h>

Move3D::Move3D() {
	_x = _y = _theta = _id = 0;
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

Speed3D Move3D::toSpeed3D(float time) {
	Speed3D speed = Speed3D(getX()/time,
								 getY()/time,
								 getTheta()/time);
	return speed;
}
