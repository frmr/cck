#ifndef CCK_BOUNDING_BOX_H
#define CCK_BOUNDING_BOX_H

#include "cckGeoCoord.h"

namespace cck
{
	class BoundingBox
	{
	public:
		const cck::GeoCoord minCoord;
		const cck::GeoCoord maxCoord;

	public:
		bool Contains( const double latitude, const double longitude ) const;
		bool Contains( const cck::GeoCoord& coord ) const;

	public:
		BoundingBox( const cck::GeoCoord& minCoord, const cck::GeoCoord& maxCoord );
	};
}

#endif // CCK_BOUNDING_BOX_H
