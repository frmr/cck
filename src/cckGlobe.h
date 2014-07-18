#ifndef CCK_GLOBE_H
#define CCK_GLOBE_H

#include <vector>

#include "cckGeoCoord.h"
#include "cckError.h"

using std::vector;

namespace cck
{
	class Globe
	{
	private:

		enum class TerrainType

		class Node
		{
		private:
			size_t			id;
			ckk::GeoCoord	loc;
			double			radius;
		};

	private:
		vector<Node>	continents;

	private:
		double	Distance( const Vec3 &pointA, const Vec3 &pointB )const;

	public:
		cck::Error	AddNode( const size_t &id, const double &latitude, const double &longitude, )

	public:
		Globe( const int seed, const double &radius );
	};
}
#endif // CCK_GLOBE_H
