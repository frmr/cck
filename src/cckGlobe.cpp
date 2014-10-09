#include "cckGlobe.h"

#include "cckMath.h"

#include <cmath>
#include <limits>
#include <map>

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

void cck::Globe::Edge::AddSides()
{
	sides.push_back( std::make_shared<Side>( nodeA, nodeB, shared_from_this() ) );
	sides.push_back( std::make_shared<Side>( nodeB, nodeA, shared_from_this() ) );
}

void cck::Globe::Edge::GetContinentData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	if ( PointOnFreeSide( point ) )
	{
		const cck::Vec3 closest = ClosestPoint( point );

		if ( Contains( closest ) )
		{
			const cck::GeoCoord closestCoord = closest.ToGeographic();
			const double maxDist = nodeA->radius + ( nodeB->radius - nodeA->radius ) * cck::Distance( nodeA->coord, closestCoord, globeRadius ) / length;
			const double distance = cck::Distance( coord, closestCoord, globeRadius );
			if ( distance < maxDist )
			{
				const double influence = 1.0 - ( distance / maxDist );
				height = influence;
				id = 1;
			}
		}
	}
}

void cck::Globe::Edge::GetMountainData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	if ( PointOnFreeSide( point ) )
	{
		const cck::Vec3 closest = ClosestPoint( point );

		if ( Contains( closest ) )
		{
			const cck::GeoCoord closestCoord = closest.ToGeographic();
			const double maxDist = nodeA->radius + ( nodeB->radius - nodeA->radius ) * cck::Distance( nodeA->coord, closestCoord, globeRadius ) / length;
			const double distance = cck::Distance( coord, closestCoord, globeRadius );
			if ( distance < maxDist )
			{
				const double influence = 1.0 - ( distance / maxDist );
				height = influence;
				id = 1;
			}
		}
	}
}

//double cck::Globe::Edge::GetInfluence( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius ) const
//{
//	const cck::Vec3 closest = ClosestPoint( point.Unit() );
//
//	if ( Contains( closest ) )
//	{
//		const cck::GeoCoord closestCoord = closest.ToGeographic();
//		const double maxDist = nodeA->radius + ( nodeB->radius - nodeA->radius ) * cck::Distance( nodeA->coord, closestCoord, globeRadius ) / length;
//		const double influence = cck::Distance( coord, closestCoord, globeRadius ) / maxDist;
//		return influence <= 1.0 ? influence : 0.0;
//	}
//	else
//	{
//		return 0.0;
//	}
//}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius )
	:	nodeA( nodeA ),
		nodeB( nodeB ),
		length( cck::Distance( nodeA->coord, nodeB->coord, globeRadius ) ),
		centerNode( std::make_shared<Node>( ( ( nodeA->coord.ToCartesian( globeRadius ) + nodeB->coord.ToCartesian( globeRadius ) ) * 0.5f ).ToGeographic(), globeRadius, mountainHeight, mountainRadius, mountainPlateau ) ),
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() )
{
	//Link two new nodes to centerNode
	auto positiveNode = std::make_shared<Node>( ( centerNode->position + normal ).ToGeographic(), globeRadius, mountainHeight, mountainRadius, mountainPlateau );
	auto negativeNode = std::make_shared<Node>( ( centerNode->position - normal ).ToGeographic(), globeRadius, mountainHeight, mountainRadius, mountainPlateau );
	//Globe::LinkNodes
}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double globeRadius )
	:	nodeA( nodeA ),
		nodeB( nodeB ),
		length( cck::Distance( nodeA->coord, nodeB->coord, globeRadius ) ),
		centerNode( nullptr ), //mountain Edge has no center Node
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() )
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

void cck::Globe::Node::GetData( const cck::GeoCoord &pointCoord, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	const double distance = cck::Distance( coord, pointCoord, globeRadius );
	if ( distance < radius )
	{
		sampleHeight = 1.0 - distance / radius;
		sampleId = 1;
	}
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

cck::Globe::Node::Node( const int id, const cck::GeoCoord& coord, const double globeRadius, const double height, const double radius )
	:	id( id ),
		coord( coord ),
		position( coord.ToCartesian( globeRadius ) ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( 0.0 )

{
}

cck::Globe::Node::Node( const cck::GeoCoord& coord, const double globeRadius, const double height, const double radius, const double plateau )
	:	id( -1 ),
		coord( coord ),
		position( coord.ToCartesian( globeRadius ) ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( plateau )

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

void cck::Globe::Triangle::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	//dist to each node
//	std::map<shared_ptr<Node>,double> nodeDistances;
//
//	for ( const auto& node : nodes )
//	{
//		nodeDistances.insert( std::pair<shared_ptr<Node>,double>( node, cck::Distance( node, coord, globeRadius ) ) ); //TODO: auto?
//	}
//
//	//for each side, calculate height
//	double highest = 0.0;
//
//	for ( const auto& side : sides )
//	{
//		double distA = nodeDistances[side->edge->nodeA];
//		double distB = node
//		double proportion = fabs( nodeDistances[side->edge->nodeA] / )
//	}

	if ( Contains( point ) )
	{
		height = 1.0;
		id = 1;
	}



	//return average height
}

//int cck::Globe::Triangle::GetNodeId( const cck::GeoCoord& coord, const double globeRadius ) const
//{
//	int closestNode = -1;
//	double closestDist = std::numeric_limits<double>::max();
//
//    for ( const auto& node : nodes )
//	{
//		double dist = node->radius * Distance( node->coord, coord, globeRadius );
//		if (  dist < closestDist )
//		{
//			closestNode = node->id;
//			closestDist = dist;
//		}
//	}
//	return closestNode;
//}

cck::Globe::Triangle::Triangle( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC, const vector<shared_ptr<Side>>& sides )
	: sides( sides )
{
	nodes.push_back( nodeA );
	nodes.push_back( nodeB );
	nodes.push_back( nodeC );
}

void cck::Globe::Section::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	double highest = std::numeric_limits<double>::min();

	for ( const auto& edge : mountainEdges )
	{
		edge->GetMountainData( coord, point, globeRadius, height, id );
		if ( height > highest )
		{
			highest = height;
		}
	}

	if ( highest == std::numeric_limits<double>::min() )
	{
		for ( const auto& node : mountainNodes )
		{
			node->GetData( coord, globeRadius, height, id );
			if ( height > highest )
			{
				highest = height;
			}
		}
	}

	height = highest;
	id = baseNode->id;
}

cck::Globe::Section::Section( const shared_ptr<Node>& baseNode, const vector<shared_ptr<Node>>& mountainNodes, const vector<shared_ptr<Edge>>& mountainEdges )
	:	baseNode( baseNode ),
		mountainNodes( mountainNodes ),
		mountainEdges( mountainEdges )
{
}

cck::LinkError cck::Globe::LinkNodes( const int nodeIdA, const int nodeIdB, const double mountainHeight, const double mountainRadius, const double mountainPlateau )
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
	shared_ptr<Edge> tempEdge( new Edge( nodePtrA, nodePtrB, mountainHeight, mountainRadius, mountainPlateau, globeRadius ) );
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

cck::NodeError cck::Globe::AddNode( const int id, const double latitude, const double longitude, const double height, const double nodeRadius )
{
	return AddNode( id, cck::GeoCoord( latitude * cck::pi / 180.0, longitude * cck::pi / 180.0 ), height, nodeRadius );
}

cck::NodeError cck::Globe::AddNode( const int id, const cck::GeoCoord& coord, const double height, const double nodeRadius )
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

	if ( height < 0.0 )
	{
		return cck::NodeError::NEGATIVE_HEIGHT;
	}

	if ( nodeRadius < 0.0 )
	{
		return cck::NodeError::NEGATIVE_RADIUS;
	}
	else if ( nodeRadius > cck::pi * globeRadius )
	{
		return cck::NodeError::DIAMETER_EXCEEDS_SPHERE_CIRCUMFERENCE;
	}

	nodes.push_back( std::make_shared<Node>( id, coord, globeRadius, height, nodeRadius ) );

	return cck::NodeError::SUCCESS;
}
//
//void cck::Globe::GetHeight( const cck::GeoCoord& coord, double& height ) const
//{
//	cck::Vec3 coordPoint = coord.ToCartesian( globeRadius );
//
//	double mostInfluence = 0.0;
//	bool inTriangle = false;
//
//	for ( const auto& triangle : triangles )
//	{
//		if ( triangle->Contains( coordPoint ) )
//		{
//			mostInfluence = 1.0;
//			inTriangle = true;
//			break;
//		}
//	}
//
//	if ( !inTriangle )
//	{
//		for ( const auto& edge : edges )
//		{
//			if ( edge->PointOnFreeSide( coordPoint ) )
//			{
//				const cck::Vec3 closestPoint = edge->ClosestPoint( coordPoint );
//				if ( edge->Contains( closestPoint ) )
//				{
//					const double dist = cck::Distance( coord, closestPoint.ToGeographic(), globeRadius );
//					if ( dist < 2000.0 )
//					{
//						const double influence = 1.0 - ( dist / 2000.0 );
//						if ( influence > mostInfluence )
//						{
//							mostInfluence = influence;
//						}
//					}
//				}
//			}
//		}
//
//		for ( const auto& node : nodes )
//		{
//			const double dist = cck::Distance( coord, node->coord, globeRadius );
//			if ( dist < 2000.0 )
//			{
//				const double influence = 1.0 - ( dist / 2000.0 );
//				if ( influence > mostInfluence )
//				{
//					mostInfluence = influence;
//				}
//			}
//		}
//	}
//
//	mostInfluence *= simplex.ScaledOctaveNoise( coordPoint.x, coordPoint.y, coordPoint.z, 7, 0.6, 0.0001, 0.0, 1.0 );
//
//	if ( mostInfluence >= 0.4 )
//	{
//		height = ( mostInfluence - 0.4 ) / 0.6;
//	}
//}
//
//int cck::Globe::GetNodeId( const cck::GeoCoord& coord ) const
//{
//	cck::Vec3 coordVec = coord.ToCartesian( globeRadius ).Unit();
//
//	for ( const auto& triangle : triangles )
//	{
//		if ( triangle->Contains( coordVec ) )
//		{
//			return triangle->GetNodeId( coord, globeRadius );
//		}
//	}
//	return -1;
//}

void cck::Globe::GetData( const double latitude, const double longitude, double& height, int& id ) const
{
	GetData( cck::GeoCoord( latitude * cck::pi / 180.0, longitude * cck::pi / 180.0 ), height, id );
}

void cck::Globe::GetData( const cck::GeoCoord& coord, double& height, int& id ) const
{
	cck::Vec3 point = coord.ToCartesian( globeRadius );

	height = std::numeric_limits<double>::min();
	id = -1;

	bool inTriangle = false;

	for ( const auto& triangle : triangles )
	{
		triangle->GetData( coord, point, globeRadius, height, id );
		if ( height > std::numeric_limits<double>::min() )
		{
			inTriangle = true;
		}
	}

	if ( !inTriangle )
	{
		double highestHeight = std::numeric_limits<double>::min();
		int	highestId = -1;

		for ( const auto& edge : edges )
		{
			double tempHeight = std::numeric_limits<double>::min();
			int tempId = -1;

			edge->GetContinentData( coord, point, globeRadius, tempHeight, tempId );

			if ( tempHeight > highestHeight )
			{
				highestHeight = tempHeight;
				highestId = tempId;
			}
		}


		for ( const auto& node : nodes )
		{
			double tempHeight = std::numeric_limits<double>::min();
			int tempId = -1;

			node->GetData( coord, globeRadius, tempHeight, tempId );

			if ( tempHeight > highestHeight )
			{
				highestHeight = tempHeight;
				highestId = tempId;
			}
		}

		height = highestHeight;
		id = highestId;
	}

	if ( height < 0.0 ) height = 0.0;

	height *= simplex.ScaledOctaveNoise( point.x, point.y, point.z, 7, 0.6, 0.0001, 0.0, 1.0 );

	if ( height >= 0.4 )
	{
		height = ( height - 0.4 ) / 0.6;
	}
	else
	{
		height = 0.0;
	}
}

cck::Globe::Globe( const double globeRadius, const unsigned int seed )
	:	globeRadius( globeRadius ),
		simplex( seed )
{
}
