/*
 * Move3D.h
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#ifndef UTILITY_Move3D_H_
#define UTILITY_Move3D_H_
#include "Speed3D.h"

class Move3D {
public:
	Move3D();
	virtual ~Move3D();

	Move3D(float x, float y, float theta);

	Speed3D toSpeed3D(float time);

	float getTheta() const {
		return _theta;
	}

	void setTheta(float theta) {
		_theta = theta;
	}

	float getX() const {
		return _x;
	}

	void setX(float x) {
		_x = x;
	}

	float getY() const {
		return _y;
	}

	void setY(float y) {
		_y = y;
	}

	Move3D operator+ (const Move3D& other)
	{
		Move3D result(getX() + other.getX(),
						getY() + other.getY(),
						getTheta() + other.getTheta());
		return result;
	}

	Move3D operator- (const Move3D& other)
	{
		Move3D result(getX() - other.getX(),
						getY() - other.getY(),
						getTheta() - other.getTheta());
		return result;
	}

	int getId() const {
		return _id;
	}

	void setId(int id) {
		_id = id;
	}

private:
	int _id;

	float _x;
	float _y;
	float _theta;
};

#endif /* UTILITY_Move3D_H_ */
