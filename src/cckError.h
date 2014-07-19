#ifndef CCK_ERROR_H
#define CCK_ERROR_H

namespace cck
{
	enum class NodeError
	{
		SUCCESS,
		ID_ALREADY_IN_USE,
		LATITUDE_OUT_OF_RANGE,
		LONGITUDE_OUT_OF_RANGE,
		NEGATIVE_RADIUS,
		DIAMETER_EXCEEDS_SPHERE_CIRCUMFERENCE
	};

	enum class LinkError
	{
		SUCCESS,
		DUPLICATE_ID,
		ID_NOT_FOUND,
		NODES_ALREADY_LINKED
	};
}

#endif // CCK_ERROR_H
