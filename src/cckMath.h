#ifndef CCK_MATH_H
#define CCK_MATH_H

#include "cckVec3.h"

namespace cck
{
	static constexpr double pi = 3.141592653589793238462643383279502884;

	cck::Vec3 CrossProduct( const cck::Vec3 &vecA, const cck::Vec3 &vecB );
}

#endif // CCK_MATH_H
