#include "quaternion.h"


quaternion::quaternion()
{
	values[0] = values[1] = values[2] = 0;
	values[3] = 1;
}

quaternion::quaternion(Vec3 v, double w)
{
	values[0] = v.x;
	values[1] = v.y;
	values[2] = v.z;
	values[3] = w;
}

quaternion::quaternion(float* axis, double angle) {
	double s = sin(angle / 2);
	double n = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	values[0] = (axis[0] * s)/n;
	values[1] = (axis[1] * s)/n;
	values[2] = (axis[2] * s)/n;
	values[3] = cos(angle / 2);
}

Vec3 quaternion::complex() 
{
	return Vec3(values[0], values[1], values[2]);
}

double quaternion::normalize() 
{
	return sqrt(values[0]*values[0] + values[1] * values[1] + values[2] * values[2] + values[3] * values[3]);
}

quaternion quaternion::normalized() {
	quaternion q;

	q.values[0] = values[0] / normalize();
	q.values[1] = values[1] / normalize();
	q.values[2] = values[2] / normalize();
	q.values[3] = values[3] / normalize();

	return q;
}

double quaternion::dot(quaternion rhs) 
{
	return values[0] * rhs.values[0] + values[1] * rhs.values[1] + values[2] * rhs.values[2] + values[3] * rhs.values[3];
}

quaternion quaternion::conjugate()
{
	return quaternion(Vec3(-values[0], -values[1], -values[2]), values[3]);
}

quaternion quaternion::operator*(quaternion rhs) 
{
	quaternion t;
	t.values[0] = values[1] * rhs.values[2] - values[2] * rhs.values[1] + values[0] * rhs.values[3] + values[3] * rhs.values[0];
	t.values[1] = values[2] * rhs.values[0] - values[0] * rhs.values[2] + values[1] * rhs.values[3] + values[3] * rhs.values[1];
	t.values[2] = values[0] * rhs.values[1] - values[1] * rhs.values[0] + values[2] * rhs.values[3] + values[3] * rhs.values[2];
	t.values[3] = values[3] * rhs.values[3] - values[0] * rhs.values[0] - values[1] * rhs.values[1] - values[2] * rhs.values[2];
	return t;
}

quaternion quaternion::operator-(quaternion rhs)
{
	quaternion t;
	t.values[0] = values[0] - rhs.values[0];
	t.values[1] = values[1] - rhs.values[1];
	t.values[2] = values[2] - rhs.values[2];
	t.values[3] = values[3] - rhs.values[3];
	return t;
}

quaternion quaternion::operator*(float rhs) 
{
	quaternion t;
	t.values[0] = values[0] * rhs;
	t.values[1] = values[1] * rhs;
	t.values[2] = values[2] * rhs;
	t.values[3] = values[3] * rhs;
	return t;
}

quaternion quaternion::operator+(quaternion rhs)
{
	quaternion t;
	t.values[0] = values[0] + rhs.values[0];
	t.values[1] = values[1] + rhs.values[1];
	t.values[2] = values[2] + rhs.values[2];
	t.values[3] = values[3] + rhs.values[3];
	return t;
}

Vec3 quaternion::rotate(Vec3 v) 
{
	return (((*this) * quaternion(v, 0)) * conjugate()).complex();
}

Vec3 quaternion::euler() 
{
	Vec3 euler;
	double ww, xx, yy, zz;

	ww = values[3] * values[3];
	xx = values[0] * values[0];
	yy = values[1] * values[1];
	zz = values[2] * values[2];

	euler.y = asin(2.0 * (values[3] * values[1] - values[0] * values[2]));
	if ((3.14159265f / 2) - fabs(euler.y) > 1e-10) {
		euler.z = atan2(2.0 * (values[0] * values[1] + values[3] * values[2]),
			xx - yy - zz + ww);
		euler.x = atan2(2.0 * (values[3] * values[0] + values[1] * values[2]),
			ww - xx - yy + zz);
	}
	else {
		euler.z = atan2(2 * values[1] * values[2] - 2 * values[0] * values[3],
			2 * values[0] * values[2] + 2 * values[1] * values[3]);
		euler.x = 0.0;

		if (euler.y < 0)
			euler.z = 3.14159265f - euler.z;
	}
	return euler;
}

float* quaternion::matrix() 
{
	float mat[16];
	mat[0] = 1 - 2 * (values[1] * values[1] + values[2] * values[2]);
	mat[1] = 2 * (values[0] * values[1] + values[2] * values[3]);
	mat[2] = 2 * (values[0] * values[2] - values[1] * values[3]);
	mat[3] = 0;
	mat[4] = 2 * (values[0] * values[1] - values[2] * values[3]);
	mat[5] = 1 - 2 * (values[0] * values[0] + values[2] * values[2]);
	mat[6] = 2 * (values[1] * values[2] + values[0] * values[3]);
	mat[7] = 0;
	mat[8] = 2 * (values[0] * values[2] + values[1] * values[3]);
	mat[9] = 2 * (values[1] * values[2] - values[0] * values[3]);
	mat[10] = 1 - 2 * (values[0] * values[0] + values[1] * values[1]);
	mat[11] = 0;
	mat[12] = 0;
	mat[13] = 0;
	mat[14] = 0;
	mat[15] = 1;

	return mat;
}
