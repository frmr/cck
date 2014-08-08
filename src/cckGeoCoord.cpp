#include "cckGeoCoord.h"

#include "cckMath.h"

#include <cmath>

cck::Vec3 cck::GeoCoord::ToCartesian( const double radius ) const
{
	return cck::Vec3( cos( lonRadians ), sin( lonRadians ), tan( latRadians ) ).Unit() * radius;
}

cck::GeoCoord::GeoCoord( const double latDegrees, const double lonDegrees )
	:	latRadians( latDegrees * cck::pi / 180.0 ),
		lonRadians( lonDegrees * cck::pi / 180.0 )

{
}
