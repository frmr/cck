#include "cckMath.h"

cck::Vec3 cck::CrossProduct( const cck::Vec3 &vecA, const cck::Vec3 &vecB )
{
	return cck::Vec3( vecA.y * vecB.z - vecA.z * vecB.y,
						vecA.z * vecB.x - vecA.x * vecB.z,
						vecA.x * vecB.y - vecA.y * vecB.x );
}
