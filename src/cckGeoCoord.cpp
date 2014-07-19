#include "cckGeoCoord.h"

#include "cckMath.h"

//cck::Vec3 ToCartesian() const
//{

//}

cck::GeoCoord::GeoCoord( const double latDegrees, const double lonDegrees )
	:	latDegrees( latDegrees ),
		lonDegrees( lonDegrees ),
		latRadians( latDegrees * cck::pi / 180.0 ),
		lonRadians( lonDegrees * cck::pi / 180.0 )

{
}
