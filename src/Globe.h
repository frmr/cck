#ifndef CCK_GLOBE_H
#define CCK_GLOBE_H

#include <vector>

#include "GeoCoord.h"

using std::vector;

namespace cck
{
	class Globe
	{
	private:

		class Continent
		{
		private:

			class Region
			{
			private:
				int				id;
				ckk::GeoCoord	center;
				double			radius;
			};

		private:
			int	id;
			vector<Region>	regions;
		};

	private:
		vector<Continent>	continents;

	public:
		bool	AddContinent( const int id, const ckk::GeoCoord &center, const double radius );
		bool	AddRegion( const int continentId, const int regionId, )
	};
}
#endif // CCK_GLOBE_H
