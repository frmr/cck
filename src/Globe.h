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
		bool	AddNode( const size_t &id, const double &latitude, )

	public:
		Globe( const int seed, const double &radius );
	};
}
#endif // CCK_GLOBE_H
