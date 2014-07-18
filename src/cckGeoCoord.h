#ifndef CCK_GEO_COORD_H
#define CCK_GEO_COORD_H

#include "cckVec3.h"

namespace cck
{
	class GeoCoord
	{
	private:
		cck::Vec3 ToCartesian() const;

	public:
		double latitude;
		double longitude;

	public:
		GeoCoord( const double latitude, const double longitude );
	};
}

#endif // CCK_GEO_COORD_H
