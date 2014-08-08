#include "cckGlobe.h"

#include <cmath>
#include <limits>
#include "cckMath.h"

void cck::Globe::Edge::AddSides()
{
	sides.push_back( std::make_shared<Side>( nodeA, nodeB, shared_from_this() ) );
	sides.push_back( std::make_shared<Side>( nodeB, nodeA, shared_from_this() ) );
}

//cck::Vec3 cck::Globe::Edge::CalculateClosestPoint( const cck::Vec3& point ) const
//{
//	cck::Vec3 normal = GetNormal();
//
//	double multiplier = ( nodeA->unitVec.z * point.x * nodeB->unitVec.y - nodeA->unitVec.z * point.y * nodeB->unitVec.x - point.x * nodeA->unitVec.y * nodeB->unitVec.z + point.y * nodeA->unitVec.x * nodeB->unitVec.z - point.z * nodeA->unitVec.x * nodeB->unitVec.y + point.z * nodeB->unitVec.x * nodeA->unitVec.y ) /
//						( nodeA->unitVec.z * normal.y * nodeB->unitVec.x - nodeA->unitVec.z * nodeB->unitVec.y * normal.x - normal.y * nodeA->unitVec.x * nodeB->unitVec.z + normal.z * nodeA->unitVec.x * nodeB->unitVec.y - normal.z * nodeB->unitVec.x * nodeA->unitVec.y + normal.x * nodeA->unitVec.y * nodeB->unitVec.z );
//
//	return cck::Vec3( point.x + multiplier * normal.x, point.y + multiplier * normal.y, point.z + multiplier * normal.z ).ToGeoCoord();
//}


cck::GeoCoord cck::Globe::Edge::CalculateClosestPoint( const cck::Vec3& point ) const
{
	cck::Vec3 normal = GetNormal();
	double multiplier = cck::DotProduct( point, normal );
	return cck::Vec3( point - normal * multiplier ).ToGeographic();
}


cck::Vec3 cck::Globe::Edge::GetNormal() const
{
	return (*sides.begin())->normal;
}

bool cck::Globe::Edge::PointOnFreeSide( const cck::Vec3& unitVec ) const
{
	for ( const auto& side : sides )
	{
		if ( !side->FormsTriangle() )
		{
			if ( cck::DotProduct( unitVec, side->normal ) >= 0.0 )
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
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ) ),
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
	return AddNode( id, cck::GeoCoord( latitude, longitude ), nodeRadius );
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

double cck::Globe::GetHeight( const double latitude, const double longitude ) const
{
	return GetHeight( cck::GeoCoord( latitude, longitude ) );
}

double cck::Globe::GetHeight( const cck::GeoCoord& coord ) const
{
//	vector<double> heights;
//
//	for ( auto nodeIt : nodes )
//	{
//		double dist = Distance( coord, nodeIt->coord );
//		if ( dist < nodeIt->radius )
//		{
//			double prop = dist / nodeIt->radius;
//			//height += ( 1.0 - ( prop * prop ) );
//			//height += 1.0 - prop;
//			heights.push_back( 1.0 - prop );
//		}
//	}
//
//	double total = 0.0;
//
//	for ( auto height : heights )
//	{
//		total += height;
//	}
//
//	if ( !heights.empty() )
//	{
//		return total;// / heights.size();
//	}
//	else
//	{
//		return 0.0;
//	}

	cck::Vec3 coordVec = coord.ToCartesian( globeRadius ).Unit();

	for ( const auto& triangle : triangles )
	{
		if ( triangle->Contains( coordVec ) )
		{
			return 1.0;
		}
	}

	for ( const auto& edge : edges )
	{
		if ( edge->PointOnFreeSide( coordVec ) )
		{

		}
	}


	return 0.0;

}

int cck::Globe::GetNodeId( const double latitude, const double longitude ) const
{
	return GetNodeId( cck::GeoCoord( latitude, longitude ) );
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

cck::Globe::Globe( const int seed, const double globeRadius )
	:	globeRadius( globeRadius )
{

}
