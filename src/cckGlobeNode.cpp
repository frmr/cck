#include "cckGlobe.h"
#include "cckMath.h"

void cck::Globe::Node::AddLink( const shared_ptr<Link>& newLink )
{
	links.push_back( newLink );
}

void cck::Globe::Node::AddToSegment( const shared_ptr<Edge>& newEdge )
{
	if ( segment == nullptr )
	{
		segment = std::make_shared<Segment>( shared_from_this() );
	}
	segment->AddEdge( newEdge );
}

void cck::Globe::Node::AddToSegment( const shared_ptr<Node>& newNode )
{
	if ( segment == nullptr )
	{
		segment = std::make_shared<Segment>( shared_from_this() );
	}
	segment->AddNode( newNode );
}

vector<shared_ptr<cck::Globe::Node>> cck::Globe::Node::FindCommonNeighbors( const shared_ptr<cck::Globe::Node>& refNode )
{
	vector<shared_ptr<Node>> commonNeighbors;

	for ( const auto& link : links )
	{
		if ( refNode->LinkedTo( link->target->id ) )
		{
			commonNeighbors.push_back( link->target );
		}
	}
	return commonNeighbors;
}

double cck::Globe::Node::GetInfluence( const cck::GeoCoord& sampleCoord, const double globeRadius ) const
{
	const double distance = cck::Distance( coord, sampleCoord, globeRadius );
	if ( distance <= radius )
	{
		double fraction = distance / radius;
		return 1.0 - ( fraction * fraction );
	}
	return 0.0;
}

shared_ptr<cck::Globe::Link> cck::Globe::Node::GetLinkTo( const int targetId ) const
{
	for ( const auto& link : links )
	{
		if ( link->target->id == targetId )
		{
			return link;
		}
	}
	return nullptr;
}

double cck::Globe::Node::GetMountainHeight( const cck::GeoCoord& sampleCoord, const double globeRadius, const double segmentHeight ) const
{
	const double distance = cck::Distance( coord, sampleCoord, globeRadius );

	if ( distance <= radius )
	{
		if ( distance <= plateau )
		{
			return height;
		}
		else
		{
			return cck::Globe::CalculateMountainHeight( segmentHeight, height, radius, plateau, distance );
		}
	}
	return 0.0;
}

shared_ptr<cck::Globe::Segment> cck::Globe::Node::GetSegment() const
{
	return segment;
}

bool cck::Globe::Node::LinkedTo( const int nodeId ) const
{
	for ( const auto& linkIt : links )
	{
		if ( linkIt->LinksTo( nodeId ) )
		{
			return true;
		}
	}
	return false;
}

void cck::Globe::Node::SampleData( double& sampleHeight, int& sampleId ) const
{
	sampleHeight = height;
	sampleId = id;
}

cck::Globe::Node::Node( const int id, const cck::GeoCoord& coord, const double height, const double radius, const double globeRadius )
	:	segment( nullptr ),
		id( id ),
		coord( coord ),
		position( coord.ToCartesian( globeRadius ) ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( 0.0 )
{
}

cck::Globe::Node::Node( const cck::Vec3& position, const double height, const double radius, const double plateau )
	:	segment( nullptr ),
		id( -1 ),
		coord( position.ToGeographic() ),
		position( position ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( plateau )
{
}
