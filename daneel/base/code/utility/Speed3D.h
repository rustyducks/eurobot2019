/*
 * Speed3D.h
 *
 *  Created on: 17 nov. 2017
 *      Author: fabien
 */

#ifndef UTILITY_SPEED3D_H_
#define UTILITY_SPEED3D_H_

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

	int getId() const {
		return _id;
	}

	void setId(int id) {
		_id = id;
	}

private:
	int _id;

	float _vx;
	float _vy;
	float _omega;
};

inline float Speed3D::getOmega() const {
	return _omega;
}

#endif /* UTILITY_SPEED3D_H_ */
