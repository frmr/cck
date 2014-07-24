#include "cckVec3.h"

#include <cmath>

cck::Vec3 cck::Vec3::Unit()
{
	double length = sqrt( x * x + y * y + z * z );
	x /= length;
	y /= length;
	z /= length;

	return *this;
}

cck::Vec3 cck::Vec3::operator+( const Vec3 &rhs ) const
{
	return cck::Vec3( x + rhs.x, y + rhs.y, z + rhs.z );
}

cck::Vec3 cck::Vec3::operator-( const Vec3 &rhs ) const
{
	return cck::Vec3( x - rhs.x, y - rhs.y, z - rhs.z );
}

cck::Vec3 cck::Vec3::operator*( const double &rhs ) const
{
	return cck::Vec3( x * rhs, y * rhs, z * rhs );
}

cck::Vec3 cck::Vec3::operator/( const double &rhs ) const
{
	return cck::Vec3( x / rhs, y / rhs, z / rhs );
}

cck::Vec3::Vec3( const double &x, const double &y, const double &z )
	:	x( x ), y( y ), z( z )
{
}
