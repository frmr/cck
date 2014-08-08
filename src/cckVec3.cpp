#include "cckVec3.h"

#include "cckMath.h"
#include <cmath>

cck::Vec3 cck::Vec3::Reverse() const
{
	return cck::Vec3( -x, -y, -z );
}

cck::GeoCoord cck::Vec3::ToGeographic() const
{
	cck::Vec3 unitVec = this->Unit();
	double lonRadians = atan2( unitVec.y, unitVec.x );

	if ( lonRadians < -cck::halfPi )
	{
		lonRadians += cck::pi;
	}
	else if ( lonRadians > cck::halfPi )
	{
		lonRadians -= cck::pi;
	}

	return cck::GeoCoord( atan( unitVec.z ), lonRadians );
}

cck::Vec3 cck::Vec3::Unit() const
{
	double length = sqrt( x * x + y * y + z * z );
	return cck::Vec3( x / length, y / length, z / length );
}

cck::Vec3 cck::Vec3::operator+( const Vec3& rhs ) const
{
	return cck::Vec3( x + rhs.x, y + rhs.y, z + rhs.z );
}

cck::Vec3 cck::Vec3::operator-( const Vec3& rhs ) const
{
	return cck::Vec3( x - rhs.x, y - rhs.y, z - rhs.z );
}

cck::Vec3 cck::Vec3::operator*( const double& rhs ) const
{
	return cck::Vec3( x * rhs, y * rhs, z * rhs );
}

cck::Vec3 cck::Vec3::operator/( const double& rhs ) const
{
	return cck::Vec3( x / rhs, y / rhs, z / rhs );
}

cck::Vec3& cck::Vec3::operator+=( const Vec3& rhs )
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

cck::Vec3& cck::Vec3::operator-=( const Vec3& rhs )
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

cck::Vec3& cck::Vec3::operator*=( const double& rhs )
{
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
}

cck::Vec3& cck::Vec3::operator/=( const double& rhs )
{
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}

cck::Vec3::Vec3()
	:	x( 0.0 ), y( 0.0 ), z( 0.0 )
{
}

cck::Vec3::Vec3( const double x, const double y, const double z )
	:	x( x ), y( y ), z( z )
{
}
