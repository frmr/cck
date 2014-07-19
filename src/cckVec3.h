#ifndef CCK_VEC3_H
#define CCK_VEC3_H

namespace cck
{
	class Vec3
	{
	public:
		double	x, y, z;

	public:
		Vec3	Unit();

	public:
		Vec3( const double &x, const double &y, const double &z );
	};
}
#endif // CCK_VEC3_H
