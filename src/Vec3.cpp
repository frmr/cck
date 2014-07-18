#include "Vec3.h"

#include <cmath>

Vec3 Vec3::Unit()
{
	double length = sqrt( x * x + y * y + z * z );
	x /= length;
	y /= length;
	z /= length;

	return *this;
}

Vec3::Vec3( const double &x, const double &y, const double &z )
	:	x( x ), y( y ), z( z )
{
}
