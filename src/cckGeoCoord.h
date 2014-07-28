#ifndef CCK_GEO_COORD_H
#define CCK_GEO_COORD_H

#include "cckVec3.h"

namespace cck
{
	class GeoCoord
	{
	public:
		cck::Vec3 ToCartesian( const double radius ) const;

	public:
		const double latDegrees;
		const double lonDegrees;
		const double latRadians;
		const double lonRadians;

	public:
		GeoCoord( const double latDegrees, const double lonDegrees );
	};
}

#endif // CCK_GEO_COORD_H
