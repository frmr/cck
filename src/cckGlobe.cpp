#include "cckGlobe.h"

#include <cmath>
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

cck::Globe::Link::Link( const NodeType type, const shared_ptr<Node> nodeA, const shared_ptr<Node> nodeB )
	:	type( type )
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

cck::Globe::Node::Node( const size_t &id, const cck::GeoCoord &coord, const Vec3 &position, const NodeType type, const double &radius )
	:	id( id ),
		coord( coord ),
		position( position ),
		type( type ),
		radius( radius )
{
}

double cck::Globe::Distance( const GeoCoord &coordA, const GeoCoord &coordB ) const
{
	//find angle between points
	//return arc length
	return radius * acos( sin( coordA.latRadians ) * sin( coordB.latRadians ) + cos( coordA.latRadians ) * cos( coordB.latRadians ) * cos( coordB.lonRadians - coordA.lonRadians ) );
}

cck::LinkError cck::Globe::AddLink( const NodeType type, const size_t &nodeIdA, const size_t &nodeIdB )
{
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

    shared_ptr<Link> tempLink( new Link( type, nodePtrA, nodePtrB ) );
    nodePtrA->AddLink( tempLink );
    nodePtrB->AddLink( tempLink );

    return cck::LinkError::SUCCESS;
}

cck::NodeError cck::Globe::AddNode( const size_t &id, const double &latitude, const double &longitude, const NodeType type, const double &nodeRadius )
{
	return AddNode( id, cck::GeoCoord( latitude, longitude ), type, nodeRadius );
}

cck::NodeError cck::Globe::AddNode( const size_t &id, const cck::GeoCoord &coord, const NodeType type, const double &nodeRadius )
{
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
	else if ( nodeRadius > cck::pi * radius )
	{
		return cck::NodeError::DIAMETER_EXCEEDS_SPHERE_CIRCUMFERENCE;
	}

	//convert coord to position
	cck::Vec3 position( 0.0, 0.0, 0.0 );

	nodes.push_back( std::make_shared<Node>( id, coord, position, type, nodeRadius ) );
}

cck::Globe::Globe( const int seed, const double &radius )
	:	radius( radius )
{

}
