#include "cckGlobe.h"

#include "cckMath.h"

#include <cmath>
#include <limits>
#include <map>
#include <iostream> //TODO: Remove this

bool cck::Globe::BspTree::BspNode::AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& newEdge )
{
	if ( segment == nullptr )
	{
		if ( coord.empty() )
		{
			if ( edge == nullptr )
			{
				edge = newEdge;
				posChild = std::make_shared<BspNode>();
				negChild = std::make_shared<BspNode>();
				return true;
			}
		}
		else
		{
			if ( posChild == nullptr || negChild == nullptr )
			{
				return false;
			}

			if ( coord.front() == true )
			{
				if ( posChild != nullptr )
				{
					coord.pop();
					return posChild->AddChildren( coord, newEdge );
				}
			}
			else
			{
				if ( negChild != nullptr )
				{
					coord.pop();
					return negChild->AddChildren( coord, newEdge );
				}
			}
		}
	}
	return false;
}

bool cck::Globe::BspTree::BspNode::AddSegment( std::queue<bool>& coord, const shared_ptr<Segment>& newSegment )
{
	if ( coord.empty() )
	{
		if ( segment == nullptr )
		{
			segment = newSegment;
			return true;
		}
	}
	else
	{
		if ( coord.front() == true )
		{
			if ( posChild != nullptr )
			{
				coord.pop();
				return posChild->AddSegment( coord, newSegment );
			}
		}
		else
		{
			if ( negChild != nullptr )
			{
				coord.pop();
				return negChild->AddSegment( coord, newSegment );
			}
		}
	}
	return false;
}

void cck::Globe::BspTree::BspNode::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	if ( segment == nullptr )
	{
		if ( cck::DotProduct( edge->normal, point ) >= 0.0 )
		{
			return posChild->GetData( coord, point, globeRadius, height, id );
		}
		else
		{
			return negChild->GetData( coord, point, globeRadius, height, id );
		}
	}
	else
	{
		segment->GetData( coord, point, globeRadius, height, id );
	}
}
//
//shared_ptr<cck::Globe::Segment>	cck::Globe::BspTree::BspNode::GetSegment( const cck::Vec3& point ) const
//{
//	if ( segment == nullptr )
//	{
//		if ( cck::DotProduct( edge->normal, point ) >= 0.0 )
//		{
//			return posChild->GetSegment( point );
//		}
//		else
//		{
//			return negChild->GetSegment( point );
//		}
//	}
//	else
//	{
//		return segment;
//	}
//
//}

bool cck::Globe::BspTree::BspNode::IsComplete() const
{
	if ( segment != nullptr )
	{
		return true;
	}
	else
	{
		if ( posChild == nullptr || negChild == nullptr )
		{
			return false;
		}
		else if ( posChild->IsComplete() && negChild->IsComplete() )
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

cck::Globe::BspTree::BspNode::BspNode()
	:	edge( nullptr ),
		segment( nullptr ),
		posChild( nullptr ),
		negChild( nullptr )
{
}

bool cck::Globe::BspTree::AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& newEdge )
{
	return root.AddChildren( coord, newEdge );
}

bool cck::Globe::BspTree::AddSegment( std::queue<bool>& coord, const shared_ptr<Segment>& newSegment )
{
	return root.AddSegment( coord, newSegment );
}

void cck::Globe::BspTree::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	root.GetData( coord, point, globeRadius, height, id );
}

//shared_ptr<cck::Globe::Segment>	cck::Globe::BspTree::GetSegment( const cck::Vec3& point ) const
//{
//	return root.GetSegment( point );
//}

bool cck::Globe::BspTree::IsComplete() const
{
	return root.IsComplete();
}

cck::Globe::BspTree::BspTree()
{
}

cck::Vec3 cck::Globe::Edge::ClosestPoint( const cck::Vec3& point ) const
{
	return cck::Vec3( point - normal * cck::DotProduct( normal, point ) );
}

cck::Globe::BspTree cck::Globe::Edge::ConstructTree( const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius ) const
{
	//construct nodes either side of center node
	const auto positiveNode = std::make_shared<Node>( ( centerNode->position + normal ), mountainHeight, mountainRadius, mountainPlateau );
	const auto negativeNode = std::make_shared<Node>( ( centerNode->position - normal ), mountainHeight, mountainRadius, mountainPlateau );

	vector<shared_ptr<Node>> mountainNodes;
	vector<shared_ptr<Edge>> mountainEdges;
	mountainEdges.push_back( std::make_shared<Edge>( positiveNode, negativeNode, globeRadius ) );

	//construct BspTree
	BspTree tempTree;
	std::queue<bool> coord;
	tempTree.AddChildren( coord, mountainEdges[0] );	//TODO: Use iterators

	const bool nodeDotProduct = cck::DotProduct( mountainEdges[0]->normal, nodeA->unitVec ) >= 0.0;
	coord.push( nodeDotProduct );
	tempTree.AddSegment( coord, std::make_shared<Segment>( nodeA, mountainNodes, mountainEdges ) );

	coord.push( !nodeDotProduct );
	tempTree.AddSegment( coord, std::make_shared<Segment>( nodeB, mountainNodes, mountainEdges ) );

	return tempTree;
}

bool cck::Globe::Edge::Contains( const cck::Vec3& point ) const
{
	const double dot12 = cck::DotProduct( nodeA->unitVec, nodeB->unitVec );
	const double dot1p = cck::DotProduct( nodeA->unitVec, point );
	const double dot2p = cck::DotProduct( nodeB->unitVec, point );

	const double invDenom = 1.0 / ( dot12 * dot12 );
	const double u = ( dot1p - dot12 * dot2p ) * invDenom;
	const double v = ( dot2p - dot12 * dot1p ) * invDenom;

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

void cck::Globe::Edge::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	tree.GetData( coord, point, globeRadius, height, id );
}

double cck::Globe::Edge::GetInfluence( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius ) const
{
	if ( PointOnFreeSide( point ) )
	{
		const cck::Vec3 closest = ClosestPoint( point );

		if ( Contains( closest ) )
		{
			const cck::GeoCoord closestCoord = closest.ToGeographic();
			const double maxDist = nodeA->radius + ( nodeB->radius - nodeA->radius ) * cck::Distance( nodeA->coord, closestCoord, globeRadius ) / length;
			const double distance = cck::Distance( coord, closestCoord, globeRadius );
			if ( distance <= maxDist )
			{
				return 1.0 - ( distance / maxDist );
			}
		}
	}
	return 0.0;
}

void cck::Globe::Edge::GetMountainData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, const double segmentHeight, double& height ) const
{
	//return zero if point is outside edge boundary
	//otherwise return result of mountain function

	const cck::Vec3 closest = ClosestPoint( point );

	if ( Contains( closest ) )
	{
		const cck::GeoCoord closestCoord = closest.ToGeographic();
		const double closestDistToNodeA = cck::Distance( nodeA->coord, closestCoord, globeRadius );
		const double maxDist = nodeA->radius + ( ( nodeB->radius - nodeA->radius ) * closestDistToNodeA / length );
		const double distance = cck::Distance( coord, closestCoord, globeRadius );

		//std::cout << nodeA->radius << " " << nodeB->radius << " " << closestDistToNodeA << " " << length << " " << maxDist << " " << distance << std::endl;

		if ( distance <= maxDist )
		{
			height = 50.0;
//			const double edgeHeight = nodeA->height + ( ( nodeB->height - nodeA->height ) * closestDistToNodeA / length );
//			const double maxPlateauDist = nodeA->plateau + ( ( nodeB->plateau - nodeA->plateau ) * closestDistToNodeA / length );
//
//			if ( distance <= maxPlateauDist )
//			{
//				height = edgeHeight;
//			}
//			else
//			{
//				//height = cck::Globe::CalculateMountainHeight( maxDist, maxPlateauDist, distance, segmentHeight );
//				height = edgeHeight;
////				std::cout << height << std::endl;
//			}
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
		centerNode( std::make_shared<Node>( ( nodeA->coord.ToCartesian( globeRadius ) + nodeB->coord.ToCartesian( globeRadius ) ) * 0.5f , mountainHeight, mountainRadius, mountainPlateau ) ),
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() ),
		tree( ConstructTree( mountainHeight, mountainRadius, mountainPlateau, globeRadius ) )
{
}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double globeRadius )
	:	nodeA( nodeA ),
		nodeB( nodeB ),
		length( cck::Distance( nodeA->coord, nodeB->coord, globeRadius ) ),
		centerNode( nullptr ), //mountain Edge has no center Node
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() ),
		tree()
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

void cck::Globe::Node::AddLink( const shared_ptr<Link>& newLink )
{
	links.push_back( newLink );
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

void cck::Globe::Node::GetData( double& sampleHeight, int& sampleId ) const
{
	sampleHeight = height;
	sampleHeight = 50.0;
	sampleId = id;
	//std::cout << height << std::endl;
}

double cck::Globe::Node::GetInfluence( const cck::GeoCoord& sampleCoord, const double globeRadius ) const
{
	const double distance = cck::Distance( coord, sampleCoord, globeRadius );
	if ( distance <= radius )
	{
		return 1.0 - ( distance / radius );
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

void cck::Globe::Node::GetMountainData( const cck::GeoCoord &pointCoord, const double globeRadius, double& sampleHeight ) const
{
	const double distance = cck::Distance( coord, pointCoord, globeRadius );
	if ( distance <= radius )
	{
		//sampleHeight = 1.0 - ( distance / radius );
		sampleHeight = height * ( 1.0 - ( distance / radius ) );
	}
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

cck::Globe::Node::Node( const int id, const cck::GeoCoord& coord, const double height, const double radius, const double globeRadius )
	:	id( id ),
		coord( coord ),
		position( coord.ToCartesian( globeRadius ) ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( 0.0 )
{
}

cck::Globe::Node::Node( const cck::Vec3& position, const double height, const double radius, const double plateau )
	:	id( -1 ),
		coord( position.ToGeographic() ),
		position( position ),
		unitVec( position.Unit() ),
		height( height ),
		radius( radius ),
		plateau( plateau )
{
}

cck::Globe::BspTree cck::Globe::Triangle::ConstructTree( const double globeRadius ) const
{
	vector<shared_ptr<Edge>> mountainEdges;
	//create Edges between triangles's center node and each side's center node
	for ( const auto& side : sides )
	{
		mountainEdges.push_back( std::make_shared<Edge>( centerNode, side->edge->centerNode, globeRadius ) );
	}

    BspTree tempTree;
    std::queue<bool> coord;
    tempTree.AddChildren( coord, mountainEdges[0] ); //TODO: Use iterators

    bool nodeDotProduct = cck::DotProduct( mountainEdges[0]->normal, sides[1]->edge->centerNode->unitVec ) >= 0.0;
	coord.push( nodeDotProduct );
    tempTree.AddChildren( coord, mountainEdges[1] );

    coord.push( !nodeDotProduct );
    tempTree.AddChildren( coord, mountainEdges[2] );

	vector<shared_ptr<Segment>> segments;
	for ( const auto& node : nodes )
	{
		segments.push_back( std::make_shared<Segment>( node ) );
	}

	for ( const auto& segment : segments )
	{
		segment->AddNode( centerNode );
	}

	for ( const auto& edge : mountainEdges )
	{
        //if ( )
	}

	//for node in nodes
	//	if dot( node,


//	vector<shared_ptr<Edge>> mountainEdges0;
//	vector<shared_ptr<Edge>> mountainEdges1;
//	vector<shared_ptr<Edge>> mountainEdges2;
//
//	vector<shared_ptr<Node>> mountainNodes0;
//	vector<shared_ptr<Node>> mountainNodes1;
//	vector<shared_ptr<Node>> mountainNodes2;
//
//
//
//
//
//	//determine which node is shared between the two edges
//	shared_ptr<Segment> node1A = std::make_shared<Segment>( sides[1]->edge->nodeA,  );
//	shared_ptr<Segment> node1B = std::make_shared<Segment>( sides[1]->edge->nodeB,  );
//
//
////	coord.push( cck::DotProduct( mountainEdges[1]->normal, sides[1]->edge->nodeA ) >= 0.0 );
////	tempTree.AddSegment( coord, std::make_shared(  ) )


    return tempTree;
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

shared_ptr<cck::Globe::Node> cck::Globe::Triangle::CreateCenterNode() const
{
	//average node properties
	cck::Vec3 averagePosition;
	double averageHeight = 0.0;
	double averageRadius = 0.0;
	double averagePlateau = 0.0;

	for ( const auto& side : sides )
	{
		averagePosition += side->edge->centerNode->position;
		averageHeight += side->edge->centerNode->height;
		averageRadius += side->edge->centerNode->radius;
		averagePlateau += side->edge->centerNode->plateau;
	}
	averagePosition /= 3.0;
	averageHeight /= 3.0;
	averageRadius /= 3.0;
	averagePlateau /= 3.0;

	return std::make_shared<Node>( averagePosition, averageHeight, averageRadius, averagePlateau );
}

vector<shared_ptr<cck::Globe::Node>> cck::Globe::Triangle::CreateNodeVector( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC ) const
{
	vector<shared_ptr<Node>> tempVector;
	tempVector.push_back( nodeA );
	tempVector.push_back( nodeB );
	tempVector.push_back( nodeC );
	return tempVector;
}

bool cck::Globe::Triangle::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
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
		height = 50.0;
		id = 1;
	}



	//return average height
}

double cck::Globe::Triangle::GetInfluence( const cck::Vec3& point ) const
{
	if ( Contains( point ) )
	{
		return 1.0;
	}
	return 0.0;
}

cck::Globe::Triangle::Triangle( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC, const vector<shared_ptr<Side>>& sides, const double globeRadius )
	:	nodes( CreateNodeVector( nodeA, nodeB, nodeC ) ),
		sides( sides ),
		centerNode( CreateCenterNode() ),
		tree( ConstructTree( globeRadius ) )
{

}

void cck::Globe::Segment::AddEdge( const shared_ptr<Edge>& newEdge )
{
	mountainEdges.push_back( newEdge );
}

void cck::Globe::Segment::AddNode( const shared_ptr<Node>& newNode )
{
	mountainNodes.push_back( newNode );
}

void cck::Globe::Segment::GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const
{
	double highest = 0.0;

	for ( const auto& edge : mountainEdges )
	{
		double mountainHeight = 0.0;
		edge->GetMountainData( coord, point, globeRadius, baseNode->height, mountainHeight );

		//std::cout << height << std::endl;

		if ( mountainHeight > highest )
		{
			highest = mountainHeight;
		}
	}

	if ( highest == 0.0 )
	{
		for ( const auto& node : mountainNodes )
		{
			double mountainHeight = 0.0;
			node->GetMountainData( coord, globeRadius, mountainHeight );
			if ( mountainHeight > highest )
			{
				highest = mountainHeight;
			}
		}
	}

	if ( highest == 0.0 )
	{
		height = baseNode->height;
	}
	else
	{
		height = highest;
	}

	height = 50.0;
	id = baseNode->id;
}

cck::Globe::Segment::Segment( const shared_ptr<Node>& baseNode, const vector<shared_ptr<Node>>& mountainNodes, const vector<shared_ptr<Edge>>& mountainEdges )
	:	baseNode( baseNode ),
		mountainNodes( mountainNodes ),
		mountainEdges( mountainEdges )
{
}

cck::Globe::Segment::Segment( const shared_ptr<Node>& baseNode )
	:	baseNode( baseNode )
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
				if ( DotProduct( side->normal, average ) >= 0.0 )
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

		triangles.push_back( std::make_shared<Triangle>( nodePtrA, nodePtrB, neighbor, commonSides, globeRadius ) );
	}

	nodePtrA->AddLink( std::make_shared<Link>( nodePtrB, tempEdge ) );
	nodePtrB->AddLink( std::make_shared<Link>( nodePtrA, tempEdge ) );
	edges.push_back( tempEdge );

	return cck::LinkError::SUCCESS;
}

double cck::Globe::CalculateMountainHeight( const double radius, const double plateau, const double distance, const double height )
{
	return sin( ( ( cck::pi * ( 1.0 - ( ( distance - plateau ) / ( radius - plateau ) ) ) ) - cck::halfPi ) ) * height;
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

	nodes.push_back( std::make_shared<Node>( id, coord, height, nodeRadius, globeRadius ) );

	return cck::NodeError::SUCCESS;
}

void cck::Globe::GetData( const double latitude, const double longitude, double& height, int& id ) const
{
	GetData( cck::GeoCoord( latitude * cck::pi / 180.0, longitude * cck::pi / 180.0 ), height, id );
}

void cck::Globe::GetData( const cck::GeoCoord& coord, double& height, int& id ) const	//TODO: rename height in all functions to sampleHeight, and id to sampleId
{
	const cck::Vec3 point = coord.ToCartesian( globeRadius );
	const double noiseValue = noise.ScaledOctaveNoise( point.x, point.y, point.z, 7, 0.6, 0.0001, 0.0, 1.0 );

	if ( noiseValue < seaScale )
	{
		height = 0.0;
		id = -1;
		return;
	}

	for ( const auto& triangle : triangles )
	{
		if ( triangle->GetData( coord, point, globeRadius, height, id ) )
		{
			double influence = ( noiseValue - seaScale ) / ( 1.0 - seaScale );
			height *= influence;
			return;
		}
	}


	double highestHeight = 0.0;
	int highestId = -1;

	for ( const auto& edge : edges )
	{
		double influence = edge->GetInfluence( coord, point, globeRadius ) * noiseValue;
		if ( influence >= seaScale )
		{
			influence = ( influence - seaScale ) / ( 1.0 - seaScale );
			double tempHeight = 0.0;
			int tempId = -1;
			edge->GetData( coord, point, globeRadius, tempHeight, tempId );
			tempHeight *= influence;
			if ( tempHeight > highestHeight )
			{
				highestHeight = tempHeight;
				highestId = tempId;
			}
		}
	}

	for ( const auto& node : nodes )
	{
		double influence = node->GetInfluence( coord, globeRadius ) * noiseValue;
		if ( influence >= seaScale )
		{
			influence = ( influence - seaScale ) / ( 1.0 - seaScale );
			double tempHeight = 0.0;
			int tempId = -1;
			node->GetData( tempHeight, tempId );
			tempHeight *= influence;
			if ( tempHeight > highestHeight )
			{
				highestHeight = tempHeight;
				highestId = tempId;
			}
		}
	}

	height = highestHeight;
	id = highestId;

//	bool inTriangle = false;
//
//	//if triangles strictly cannot overlap, break from this loop when triangle found
//	for ( const auto& triangle : triangles )
//	{
//		triangle->GetData( coord, point, globeRadius, height, id );
//		if ( height > 0.0 )
//		{
//			inTriangle = true;
//		}
//	}
//
//	if ( !inTriangle )
//	{
//		double highestHeight = 0.0;
//		int	highestId = -1;
//
//		for ( const auto& edge : edges )
//		{
//			double tempHeight = 0.0;
//			int tempId = -1;
//
//			edge->GetData( coord, point, globeRadius, tempHeight, tempId );
//
//			if ( tempHeight > highestHeight )
//			{
//				highestHeight = tempHeight;
//				highestId = tempId;
//			}
//		}
//
//
//		for ( const auto& node : nodes )
//		{
//			double tempHeight = 0.0;
//			int tempId = -1;
//
//			node->GetContinentData( coord, globeRadius, tempHeight, tempId );
//
//			if ( tempHeight > highestHeight )
//			{
//				highestHeight = tempHeight;
//				highestId = tempId;
//			}
//		}
//
//		height = highestHeight;
//		id = highestId;
//	}

	//if ( height < 1.0 ) height = 0.0;

//	double noiseHeight = noise.ScaledOctaveNoise( point.x, point.y, point.z, 7, 0.6, 0.0001, 0.0, 1.0 );
//
//	if ( noiseHeight >= seaScale )
//	{
//		//scale to range 0-1
//		noiseHeight = ( noiseHeight - seaScale ) / ( 1.0 - seaScale );
//	}
//	else
//	{
//		noiseHeight = 0.0;
//	}
//
//	height *= noiseHeight;
}

cck::Globe::Globe( const double globeRadius, const unsigned int seed )
	:	globeRadius( globeRadius ),
		seaScale( 0.4 ), //TODO: Make sure seaScale is in range 0-1, and taken as a parameter
		noise( seed )
{
}
