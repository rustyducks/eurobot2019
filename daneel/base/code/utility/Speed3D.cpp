/*
 * Speed3D.cpp
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#include <Speed3D.h>

Speed3D::Speed3D() {
	_vx = _vy = _omega = 0;
}

Speed3D::~Speed3D() {
	// TODO Auto-generated destructor stub
}

Speed3D::Speed3D(float x, float y, float theta) {
	_vx = x;
	_vy = y;
	_omega = theta;
}
