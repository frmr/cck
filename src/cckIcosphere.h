#ifndef CCK_ICOSPHERE_H
#define CCK_ICOSPHERE_H

#include <memory>
#include <vector>

#include "Vec3.h"
#include "GeoCoord.h"

using std::shared_ptr;
using std::vector;

namespace cck
{
	class Icosphere
	{
	private:
		class Triangle
		{
		private:
			Vec3				center;
			shared_ptr<Vec3>	verts;

		public:
			double	DistanceToCenter( const Vec3 &point ) const;

		public:

		};
	private:
		vector<GeoCoord>			coords;	//this should be sorted
		vector<shared_ptr<Vec3>> 	verts;
		vector<shared_ptr<

	private;
		void	DefineIcosphere( vector<Vec3> &verts,  ) const;

	public:
		Icosphere( const size_t subdivisions );
}

#endif
