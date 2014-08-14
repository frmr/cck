#include "cckGlobe.h"

#include <cmath>
#include <limits>
#include "cckMath.h"

void cck::Globe::Edge::AddSides()
{
	sides.push_back( std::make_shared<Side>( nodeA, nodeB, shared_from_this() ) );
	sides.push_back( std::make_shared<Side>( nodeB, nodeA, shared_from_this() ) );
	normal = (*sides.begin())->normal;
}

cck::Vec3 cck::Globe::Edge::ClosestPoint( const cck::Vec3& point ) const
{
	return cck::Vec3( point - normal * cck::DotProduct( normal, point ) );
}

bool cck::Globe::Edge::Contains( const cck::Vec3& point ) const
{
    double dot12 = cck::DotProduct( nodeA->unitVec, nodeB->unitVec );
    double dot1p = cck::DotProduct( nodeA->unitVec, point );
    double dot2p = cck::DotProduct( nodeB->unitVec, point );

    double invDenom = 1.0 / ( dot12 * dot12 );
    double u = ( dot1p - dot12 * dot2p ) * invDenom;
    double v = ( dot2p - dot12 * dot1p ) * invDenom;

	if ( u < 0.0 || v < 0.0 )
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool cck::Globe::Edge::PointOnFreeSide( const cck::Vec3& point ) const
{
	for ( const auto& side : sides )
	{
		if ( !side->FormsTriangle() )
		{
			if ( cck::DotProduct( point, side->normal ) >= 0.0 )
			{
				return true;
			}
		}
	}

	return false;
}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double borderScale )
	:	nodeA( nodeA ),
		nodeB( nodeB ),
		borderScale( borderScale )
{
}

bool cck::Globe::Side::FormsTriangle() const
{
	return formsTriangle;
}

void cck::Globe::Side::SetFormsTriangle()
{
	formsTriangle = true;
}

cck::Globe::Side::Side( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Edge>& edge )
	:	formsTriangle( false ),
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() ),
		edge( edge )
{
}

bool cck::Globe::Link::LinksTo( const int nodeId ) const
{
	return ( target->id == nodeId ) ? true : false;
}

cck::Globe::Link::Link( const shared_ptr<Node>& target, const shared_ptr<Edge>& edge )
	:	target( target ),
		edge( edge )
{
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

void cck::Globe::Node::AddLink( const shared_ptr<Link>& newLink )
{
	links.push_back( newLink );
}

cck::Globe::Node::Node( const int id, const cck::GeoCoord& coord, const Vec3& position, const double radius )
	:	id( id ),
		coord( coord ),
		position( position ),
		unitVec( position.Unit() ),
		radius( radius )
{
}

bool cck::Globe::Triangle::Contains( const cck::Vec3& unitVec ) const
{
	for ( const auto& side : sides )
	{
		if ( DotProduct( unitVec, side->normal ) < 0.0 )
		{
			return false;
		}
	}
	return true;
}

double	cck::Globe::Triangle::GetHeight( const cck::GeoCoord& coord ) const
{
	//dist to each node
	//save in map
	//for each side, calculate height
	//return average height
}

int cck::Globe::Triangle::GetNodeId( const cck::GeoCoord& coord, const double globeRadius ) const
{
	int closestNode = -1;
	double closestDist = std::numeric_limits<double>::max();

    for ( const auto& node : nodes )
	{
		double dist = node->radius * Distance( node->coord, coord, globeRadius );
		if (  dist < closestDist )
		{
			closestNode = node->id;
			closestDist = dist;
		}
	}
	return closestNode;
}

cck::Globe::Triangle::Triangle( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC, const vector<shared_ptr<Side>>& sides )
	: sides( sides )
{
	nodes.push_back( nodeA );
	nodes.push_back( nodeB );
	nodes.push_back( nodeC );
}

cck::LinkError cck::Globe::LinkNodes( const int nodeIdA, const int nodeIdB, const double borderScale )
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

	for ( const auto& nodeIt : nodes )
	{
		if ( nodePtrA == nullptr || nodePtrB == nullptr )
		{
			if ( nodeIt->id == nodeIdA )
			{
				if ( nodeIt->LinkedTo( nodeIdB ) )
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
				if ( nodeIt->LinkedTo( nodeIdA ) )
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

	//Create temporary edge to test new Triangles
	shared_ptr<Edge> tempEdge( new Edge( nodePtrA, nodePtrB, borderScale ) );
	tempEdge->AddSides();

	//Search for common neighbors of nodeA and nodeB that form a Triangle
	vector<shared_ptr<Node>> commonNeighbors = nodePtrA->FindCommonNeighbors( nodePtrB ); //TODO: Tidy this and below loop up

	for ( const auto& neighbor : commonNeighbors )
	{
		vector<shared_ptr<Edge>> commonEdges;
		//commonEdges.push_back( nodePtrA->GetLinkTo( nodePtrB->id )->edge );
		commonEdges.push_back( tempEdge );
		commonEdges.push_back( nodePtrB->GetLinkTo( neighbor->id )->edge );
		commonEdges.push_back( neighbor->GetLinkTo( nodePtrA->id )->edge );

		cck::Vec3 average = ( nodePtrA->position + nodePtrB->position + neighbor->position ) / 3.0;
		average = average.Unit();

		vector<shared_ptr<Side>> commonSides;

		//dot product with each edge side
		for ( const auto& edge : commonEdges )
		{
			for ( const auto& side : edge->sides )
			{
				if ( DotProduct( average, side->normal ) >= 0.0 )
				{
					if ( side->FormsTriangle() )
					{
						return cck::LinkError::TRIANGLE_CONFLICT;
					}
					else
					{
						side->SetFormsTriangle();
						commonSides.push_back( side );
					}
				}
			}
		}

		triangles.push_back( std::make_shared<Triangle>( nodePtrA, nodePtrB, neighbor, commonSides ) );
	}

	nodePtrA->AddLink( std::make_shared<Link>( nodePtrB, tempEdge ) );
	nodePtrB->AddLink( std::make_shared<Link>( nodePtrA, tempEdge ) );
	edges.push_back( tempEdge );

    return cck::LinkError::SUCCESS;
}

cck::NodeError cck::Globe::AddNode( const int id, const double latitude, const double longitude, const double nodeRadius )
{
	return AddNode( id, cck::GeoCoord( latitude * cck::pi / 180.0, longitude * cck::pi / 180.0 ), nodeRadius );
}

cck::NodeError cck::Globe::AddNode( const int id, const cck::GeoCoord& coord, const double nodeRadius )
{
	if ( id < 0 )
	{
		return cck::NodeError::NEGATIVE_ID;
	}

	for ( const auto& nodeIt : nodes )
	{
		if ( nodeIt->id == id )
		{
			return cck::NodeError::ID_ALREADY_IN_USE;
		}
	}

	if ( coord.latRadians < -cck::halfPi || coord.latRadians > cck::halfPi )
	{
		return cck::NodeError::LATITUDE_OUT_OF_RANGE;
	}

	if ( coord.lonRadians < -cck::pi || coord.lonRadians > cck::pi )
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

double cck::Globe::GetHeight( const cck::GeoCoord& coord ) const
{
	cck::Vec3 coordPoint = coord.ToCartesian( globeRadius );

	double mostInfluence = 0.0;
	bool inTriangle = false;

	for ( const auto& triangle : triangles )
	{
		if ( triangle->Contains( coordPoint ) )
		{
			mostInfluence = 1.0;
			inTriangle = true;
			break;
		}
	}

	if ( !inTriangle )
	{
		for ( const auto& edge : edges )
		{
			if ( edge->PointOnFreeSide( coordPoint ) )
			{
				const cck::Vec3 closestPoint = edge->ClosestPoint( coordPoint );
				if ( edge->Contains( closestPoint ) )
				{
					const double dist = cck::Distance( coord, closestPoint.ToGeographic(), globeRadius );
					if ( dist < 2000.0 )
					{
						const double influence = 1.0 - ( dist / 2000.0 );
						if ( influence > mostInfluence )
						{
							mostInfluence = influence;
						}
					}
				}
			}
		}

		for ( const auto& node : nodes )
		{
			const double dist = cck::Distance( coord, node->coord, globeRadius );
			if ( dist < 2000.0 )
			{
				const double influence = 1.0 - ( dist / 2000.0 );
				if ( influence > mostInfluence )
				{
					mostInfluence = influence;
				}
			}
		}
	}

	//mostInfluence = 1.0;

	//coordPoint = coordPoint.Unit() * 256;

	//return ( mostInfluence * simplex.ScaledOctaveNoise( coordPoint.x, coordPoint.y, coordPoint.z, 4, 0.8, 0.005, 0.0, 1.0 ) >= 0.5 ) ? 1.0 : 0.0;
	//return ( mostInfluence * simplex.ScaledOctaveNoise( coordPoint.x, coordPoint.y, coordPoint.z, 7, 0.6, 0.0001, 0.0, 1.0 ) >= 0.4 ) ? 1.0 : 0.0;

	//return mostInfluence * simplex.ScaledOctaveNoise( coordPoint.x, coordPoint.y, coordPoint.z, 7, 0.6, 0.0001, 0.0, 1.0 ) >= 0.4 ? 1.0 : 0.0;

	mostInfluence *= simplex.ScaledOctaveNoise( coordPoint.x, coordPoint.y, coordPoint.z, 7, 0.6, 0.0001, 0.0, 1.0 );

	if ( mostInfluence >= 0.4 )
	{
		return ( mostInfluence - 0.4 ) / 0.6;
	}
	//if influence > 0.4, scale influence * simplex to range [0,1]
}

int cck::Globe::GetNodeId( const cck::GeoCoord& coord ) const
{
	cck::Vec3 coordVec = coord.ToCartesian( globeRadius ).Unit();

	for ( const auto& triangle : triangles )
	{
		if ( triangle->Contains( coordVec ) )
		{
			return triangle->GetNodeId( coord, globeRadius );
		}
	}
	return -1;
}

cck::Data cck::Globe::GetData( const double latitude, const double longitude ) const
{
	return GetData( cck::GeoCoord( latitude * cck::pi / 180.0, longitude * cck::pi / 180.0 ) );
}

cck::Data cck::Globe::GetData( const cck::GeoCoord& coord ) const
{
	cck::Data output;
	output.height = GetHeight( coord );
	output.id = output.height > 0.0 ? 1 : -1;
	return output;
}

cck::Globe::Globe( const double globeRadius, const unsigned int seed )
	:	globeRadius( globeRadius ),
		simplex( seed )
{
}
