#ifndef QUATERNION_H
#define QUATERNION_H

#include "simpleMath.h"

class quaternion
{
public:
	double values[4];
	quaternion();
	quaternion(Vec3 v, double w);
	quaternion(float* axis, double angle);

	Vec3  complex();

	double normalize();
	quaternion normalized();
	double dot(quaternion rhs);
	quaternion conjugate();
	quaternion operator*(quaternion q);
	quaternion operator-(quaternion q);
	quaternion operator*(float q);
	quaternion operator+(quaternion q);
	Vec3 rotate(Vec3 v);
	Vec3 euler();
	float* matrix();
};

#endif

