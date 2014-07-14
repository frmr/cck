#ifndef CCK_GEO_COORD_H
#define CCK_GEO_COORD_H

namespace cck
{
	class GeoCoord
	{
	private:
		double	latitude;
		double	longitude;

	public:
		double	GetLatitude() const;
		double	GetLongitude() const;
		void	Normalise();

	public:
		GeoCoord( const double latitude, const double longitude );
	};
}

#endif // CCK_GEO_COORD_H
