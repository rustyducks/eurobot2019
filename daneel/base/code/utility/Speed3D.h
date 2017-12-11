/*
 * Speed3D.h
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#ifndef UTILITY_SPEED3D_H_
#define UTILITY_SPEED3D_H_

#include <Arduino.h>

class Speed3D {
public:
	Speed3D();
	virtual ~Speed3D();

	Speed3D(float x, float y, float theta);

	float getOmega() const;

	void setOmega(float theta) {
		_omega = theta;
	}

	float getVx() const {
		return _vx;
	}

	void setVx(float x) {
		_vx = x;
	}

	float getVy() const {
		return _vy;
	}

	void setVy(float y) {
		_vy = y;
	}

	Speed3D operator+ (const Speed3D& other)
	{
		Speed3D result(getVx() + other.getVx(),
						getVy() + other.getVy(),
						getOmega() + other.getOmega());
		return result;
	}

	Speed3D operator- (const Speed3D& other)
	{
		Speed3D result(getVx() - other.getVx(),
						getVy() - other.getVy(),
						getOmega() - other.getOmega());
		return result;
	}

	Speed3D operator*(float k) {
		Speed3D result(k * getVx(),
				       k * getVy(),
		       		   k * getOmega());
		return result;
	}

	int getId() const {
		return _id;
	}

	void setId(int id) {
		_id = id;
	}

	void print(const char* text);

private:
	int _id;

	float _vx;
	float _vy;
	float _omega;
};

inline float Speed3D::getOmega() const {
	return _omega;
}

inline void Speed3D::print(const char* text) {
	Serial.print(text);
	Serial.print(getVx());
	Serial.print("\t");
	Serial.print(getVy());
	Serial.print("\t");
	Serial.print(getOmega());
}

#endif /* UTILITY_SPEED3D_H_ */
