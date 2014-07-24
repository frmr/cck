#include "cckGlobe.h"

#include <cmath>
#include <limits>
#include "cckMath.h"

bool cck::Globe::Link::LinksNode( const size_t &nodeId ) const
{
	for ( auto nodeIt : nodes )
	{
		if ( nodeIt->id == nodeId )
		{
			return true;
		}
	}
	return false;
}

cck::Globe::Link::Link( const shared_ptr<Node> nodeA, const shared_ptr<Node> nodeB, const double borderScale )
	:	borderScale( borderScale )
{
	nodes.push_back( nodeA );
	nodes.push_back( nodeB );
}

bool cck::Globe::Node::LinkedToNode( const size_t &nodeId ) const
{
	for ( auto linkIt : links )
	{
		if ( linkIt->LinksNode( nodeId ) )
		{
			return true;
		}
	}
	return false;
}

void cck::Globe::Node::AddLink( const shared_ptr<Link> newLink )
{
	links.push_back( newLink );
}

cck::Globe::Node::Node( const size_t &id, const cck::GeoCoord &coord, const Vec3 &position, const double &radius )
	:	id( id ),
		coord( coord ),
		position( position ),
		radius( radius )
{
}

double cck::Globe::Distance( const GeoCoord &coordA, const GeoCoord &coordB ) const
{
	return globeRadius * acos( sin( coordA.latRadians ) * sin( coordB.latRadians ) + cos( coordA.latRadians ) * cos( coordB.latRadians ) * cos( coordB.lonRadians - coordA.lonRadians ) );
}

cck::LinkError cck::Globe::AddLink( const size_t &nodeIdA, const size_t &nodeIdB, const double borderScale )
{
	if ( nodeIdA < 0 || nodeIdB < 0 )
	{
		return cck::LinkError::NEGATIVE_ID;
	}

	if ( nodeIdA == nodeIdB )
	{
		return cck::LinkError::DUPLICATE_ID;
	}

	shared_ptr<Node> nodePtrA( nullptr );
	shared_ptr<Node> nodePtrB( nullptr );

	for ( auto nodeIt : nodes )
	{
		if ( nodePtrA == nullptr || nodePtrB == nullptr )
		{
			if ( nodeIt->id == nodeIdA )
			{
				if ( nodeIt->LinkedToNode( nodeIdB ) )
				{
					return cck::LinkError::NODES_ALREADY_LINKED;
				}
				else
				{
					nodePtrA = nodeIt;
				}
			}
			else if ( nodeIt->id == nodeIdB )
			{
				if ( nodeIt->LinkedToNode( nodeIdA ) )
				{
					return cck::LinkError::NODES_ALREADY_LINKED;
				}
				else
				{
					nodePtrB = nodeIt;
				}
			}
		}
		else
		{
			break;
		}
	}

	if ( nodePtrA == nullptr || nodePtrB == nullptr )
	{
		return cck::LinkError::ID_NOT_FOUND;
	}

    shared_ptr<Link> tempLink( new Link( nodePtrA, nodePtrB, borderScale ) );
    nodePtrA->AddLink( tempLink );
    nodePtrB->AddLink( tempLink );

    return cck::LinkError::SUCCESS;
}

cck::NodeError cck::Globe::AddNode( const size_t &id, const double &latitude, const double &longitude, const double &nodeRadius )
{
	return AddNode( id, cck::GeoCoord( latitude, longitude ), nodeRadius );
}

cck::NodeError cck::Globe::AddNode( const size_t &id, const cck::GeoCoord &coord, const double &nodeRadius )
{
	if ( id < 0 )
	{
		return cck::NodeError::NEGATIVE_ID;
	}

	for ( auto nodeIt : nodes )
	{
		if ( nodeIt->id == id )
		{
			return cck::NodeError::ID_ALREADY_IN_USE;
		}
	}

	if ( coord.latDegrees < -90.0 || coord.latDegrees > 90.0 )
	{
		return cck::NodeError::LATITUDE_OUT_OF_RANGE;
	}

	if ( coord.lonDegrees < -180.0 || coord.lonDegrees > 180.0 )
	{
		return cck::NodeError::LONGITUDE_OUT_OF_RANGE;
	}

	if ( nodeRadius < 0.0 )
	{
		return cck::NodeError::NEGATIVE_RADIUS;
	}
	else if ( nodeRadius > cck::pi * globeRadius )
	{
		return cck::NodeError::DIAMETER_EXCEEDS_SPHERE_CIRCUMFERENCE;
	}

	nodes.push_back( std::make_shared<Node>( id, coord, coord.ToCartesian( globeRadius ), nodeRadius ) );

	return cck::NodeError::SUCCESS;
}

double cck::Globe::GetHeight( const double &latitude, const double &longitude ) const
{
	return GetHeight( cck::GeoCoord( latitude, longitude ) );
}

double cck::Globe::GetHeight( const cck::GeoCoord &coord ) const
{
	vector<double> heights;

	for ( auto nodeIt : nodes )
	{
		double dist = Distance( coord, nodeIt->coord );
		if ( dist < nodeIt->radius )
		{
			double prop = dist / nodeIt->radius;
			//height += ( 1.0 - ( prop * prop ) );
			//height += 1.0 - prop;
			heights.push_back( 1.0 - prop );
		}
	}

	double total = 0.0;

	for ( auto height : heights )
	{
		total += height;
	}

	if ( !heights.empty() )
	{
		return total;// / heights.size();
	}
	else
	{
		return 0.0;
	}
}

size_t cck::Globe::GetNodeId( const double &latitude, const double &longitude ) const
{
	return GetNodeId( cck::GeoCoord( latitude, longitude ) );
}

size_t cck::Globe::GetNodeId( const cck::GeoCoord &coord ) const
{
	size_t closestNode = -1;
	double closestDist = std::numeric_limits<double>::max();

	for ( auto nodeIt : nodes )
	{
		double dist = Distance( coord, nodeIt->coord );
		if ( dist < closestDist )
		{
			closestDist = dist;
			closestNode = nodeIt->id;
		}
	}
	return closestNode;
}

cck::Globe::Globe( const int seed, const double &globeRadius )
	:	globeRadius( globeRadius )
{

}
