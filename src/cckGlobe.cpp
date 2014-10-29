#include "cckGlobe.h"

#include "cckMath.h"

#include <cmath>
#include <limits>
#include <map>
#include <iostream> //TODO: Remove this

bool cck::Globe::BspTree::BspNode::AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& newEdge )
{
	if ( node == nullptr )
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

bool cck::Globe::BspTree::BspNode::AddNode( std::queue<bool>& coord, const shared_ptr<Node>& newNode )
{
	if ( coord.empty() )
	{
		if ( node == nullptr )
		{
			node = newNode;
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
				return posChild->AddNode( coord, newNode );
			}
		}
		else
		{
			if ( negChild != nullptr )
			{
				coord.pop();
				return negChild->AddNode( coord, newNode );
			}
		}
	}
	return false;
}

bool cck::Globe::BspTree::BspNode::IsComplete() const
{
	if ( node != nullptr )
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

void cck::Globe::BspTree::BspNode::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	if ( node == nullptr )
	{
		if ( cck::DotProduct( edge->normal, samplePoint ) >= 0.0 )
		{
			return posChild->SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
		}
		else
		{
			return negChild->SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
		}
	}
	else
	{
		node->GetSegment()->SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
	}
}

cck::Globe::BspTree::BspNode::BspNode()
	:	edge( nullptr ),
		node( nullptr ),
		posChild( nullptr ),
		negChild( nullptr )
{
}

bool cck::Globe::BspTree::AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& newEdge )
{
	return root.AddChildren( coord, newEdge );
}

bool cck::Globe::BspTree::AddNode( std::queue<bool>& coord, const shared_ptr<Node>& newNode )
{
	return root.AddNode( coord, newNode );
}

bool cck::Globe::BspTree::IsComplete() const
{
	return root.IsComplete();
}

void cck::Globe::BspTree::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	root.SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
}

cck::Globe::BspTree::BspTree()
{
}

cck::Vec3 cck::Globe::Edge::ClosestPoint( const cck::Vec3& samplePoint ) const
{
	return cck::Vec3( samplePoint - normal * cck::DotProduct( normal, samplePoint ) );
}

cck::Globe::BspTree cck::Globe::Edge::ConstructTree( const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius )
{
	//construct nodes either side of center node
	const auto positiveNode = std::make_shared<Node>( centerNode->position + normal * globeRadius, mountainHeight, mountainRadius, mountainPlateau );
	const auto negativeNode = std::make_shared<Node>( centerNode->position - normal * globeRadius, mountainHeight, mountainRadius, mountainPlateau );

	positiveMountain = std::make_shared<Edge>( centerNode, positiveNode, globeRadius );
	negativeMountain = std::make_shared<Edge>( centerNode, negativeNode, globeRadius );

	//construct BspTree
	BspTree tempTree;
	std::queue<bool> coord;
	tempTree.AddChildren( coord, positiveMountain );

	const bool nodeDotProduct = cck::DotProduct( positiveMountain->normal, nodeA->unitVec ) >= 0.0;

	coord.push( nodeDotProduct );
	tempTree.AddNode( coord, nodeA );
	nodeA->AddToSegment( positiveMountain );
	nodeA->AddToSegment( negativeMountain );
	nodeA->AddToSegment( centerNode );

	coord.push( !nodeDotProduct );
	tempTree.AddNode( coord, nodeB );
	nodeB->AddToSegment( positiveMountain );
	nodeB->AddToSegment( negativeMountain );
	nodeB->AddToSegment( centerNode );

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

bool cck::Globe::Edge::PointOnFreeSide( const cck::Vec3& samplePoint ) const
{
	for ( const auto& side : sides )
	{
		if ( !side->FormsTriangle() )
		{
			if ( cck::DotProduct( samplePoint, side->normal ) >= 0.0 )
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

double cck::Globe::Edge::GetInfluence( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius ) const
{
	if ( PointOnFreeSide( samplePoint ) )
	{
		const cck::Vec3 closest = ClosestPoint( samplePoint );

		if ( Contains( closest ) )
		{
			const cck::GeoCoord closestCoord = closest.ToGeographic();
			const double maxDist = nodeA->radius + ( nodeB->radius - nodeA->radius ) * cck::Distance( nodeA->coord, closestCoord, globeRadius ) / length;
			const double distance = cck::Distance( sampleCoord, closestCoord, globeRadius );
			if ( distance <= maxDist )
			{
				double fraction = distance / maxDist;
				return 1.0 - ( fraction * fraction );
			}
		}
	}
	return 0.0;
}

double cck::Globe::Edge::GetMountainHeight( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, const double segmentHeight ) const
{
	const cck::Vec3 closest = ClosestPoint( samplePoint );

	if ( Contains( closest ) )
	{
		const cck::GeoCoord closestCoord = closest.ToGeographic();
		const double closestDistToNodeA = cck::Distance( nodeA->coord, closestCoord, globeRadius );
		const double edgeRadius = nodeA->radius + ( ( nodeB->radius - nodeA->radius ) * ( closestDistToNodeA / length ) );
		const double distance = cck::Distance( sampleCoord, closestCoord, globeRadius );

		if ( distance <= edgeRadius )
		{
			const double edgeHeight = nodeA->height + ( ( nodeB->height - nodeA->height ) * ( closestDistToNodeA / length ) );
			const double edgePlateau = nodeA->plateau + ( ( nodeB->plateau - nodeA->plateau ) * ( closestDistToNodeA / length ) );

			if ( distance <= edgePlateau )
			{
				return edgeHeight;
			}
			else
			{
				return cck::Globe::CalculateMountainHeight( segmentHeight, edgeHeight, edgeRadius, edgePlateau, distance );
			}
		}
	}

	return 0.0;
}

bool cck::Globe::Edge::IsActive() const
{
	return active;
}

void cck::Globe::Edge::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	tree.SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
}

void cck::Globe::Edge::SetInactive()
{
	active = false;
}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius )
	:	active( true ),
		positiveMountain( nullptr ),
		negativeMountain( nullptr ),
		nodeA( nodeA ),
		nodeB( nodeB ),
		length( cck::Distance( nodeA->coord, nodeB->coord, globeRadius ) ),
		centerNode( std::make_shared<Node>( ( nodeA->coord.ToCartesian( globeRadius ) + nodeB->coord.ToCartesian( globeRadius ) ) * 0.5f , mountainHeight, mountainRadius, mountainPlateau ) ),
		normal( cck::CrossProduct( nodeA->unitVec, nodeB->unitVec ).Unit() ),
		tree( ConstructTree( mountainHeight, mountainRadius, mountainPlateau, globeRadius ) )
{
}

cck::Globe::Edge::Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double globeRadius )
	:	active( true ),
		positiveMountain( nullptr ),
		negativeMountain( nullptr ),
		nodeA( nodeA ),
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

cck::Globe::BspTree cck::Globe::Triangle::ConstructTree( const double globeRadius ) const
{
	vector<shared_ptr<Edge>> mountainEdges;
	//create Edges between triangles's center node and each side's center node
	for ( const auto& side : sides )
	{
		const auto newMountainEdge = std::make_shared<Edge>( centerNode, side->edge->centerNode, globeRadius );
		mountainEdges.push_back( newMountainEdge );
		side->edge->nodeA->AddToSegment( newMountainEdge ); //TODO: Only add the new edge to one segment; check with dot product against tangent
		side->edge->nodeB->AddToSegment( newMountainEdge );
	}

    BspTree tempTree;
    std::queue<bool> coord;
    tempTree.AddChildren( coord, mountainEdges[0] ); //TODO: Use iterators

    bool dotNode1 = cck::DotProduct( mountainEdges[0]->normal, sides[1]->edge->centerNode->unitVec ) >= 0.0;
	coord.push( dotNode1 );
    tempTree.AddChildren( coord, mountainEdges[1] );

    coord.push( !dotNode1 );
    tempTree.AddChildren( coord, mountainEdges[2] );

	for ( int sideIndex = 1; sideIndex < 3; sideIndex++ )
	{
		for ( const auto& node : nodes )
		{
			if ( node->id == sides[sideIndex]->edge->nodeA->id || node->id == sides[sideIndex]->edge->nodeB->id )
			{
				coord.push( ( sideIndex == 1 ) ? dotNode1 : !dotNode1 );
				coord.push( cck::DotProduct( mountainEdges[sideIndex]->normal, node->unitVec ) >= 0.0 );
				tempTree.AddNode( coord, node );
			}
		}
	}

    return tempTree;
}

bool cck::Globe::Triangle::Contains( const cck::Vec3& point ) const
{
	for ( const auto& side : sides )
	{
		if ( DotProduct( point, side->normal ) < 0.0 )
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

double cck::Globe::Triangle::GetInfluence( const cck::Vec3& samplePoint ) const
{
	if ( Contains( samplePoint ) )
	{
		return 1.0;
	}
	return 0.0;
}

bool cck::Globe::Triangle::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	if ( Contains( samplePoint ) )
	{
		tree.SampleData( sampleCoord, samplePoint, globeRadius, sampleHeight, sampleId );
		return true;
	}

	return false;
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

void cck::Globe::Segment::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, double& sampleHeight, int& sampleId ) const
{
	sampleId = baseNode->id;

	double highest = 0.0;

	for ( const auto& edge : mountainEdges )
	{
		if ( edge->IsActive() )
		{
			double mountainHeight = edge->GetMountainHeight( sampleCoord, samplePoint, globeRadius, baseNode->height );

			if ( mountainHeight > highest )
			{
				highest = mountainHeight;
			}
		}
	}

	if ( highest == 0.0 )
	{
		for ( const auto& node : mountainNodes )
		{
			double mountainHeight = node->GetMountainHeight( sampleCoord, globeRadius, baseNode->height );
			if ( mountainHeight > highest )
			{
				highest = mountainHeight;
			}
		}
	}

	if ( highest == 0.0 )
	{
		sampleHeight = baseNode->height;
	}
	else
	{
		sampleHeight = highest;
	}
}

cck::Globe::Segment::Segment( const shared_ptr<Node>& baseNode, const vector<shared_ptr<Node>>& mountainNodes, const vector<shared_ptr<Edge>>& mountainEdges )
	:	mountainNodes( mountainNodes ),
		mountainEdges( mountainEdges ),
		baseNode( baseNode )
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

						if ( cck::DotProduct( side->edge->normal, average ) >= 0.0 )
						{
							side->edge->positiveMountain->SetInactive();
						}
						else
						{
							side->edge->negativeMountain->SetInactive();
						}
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

double cck::Globe::CalculateMountainHeight( const double segmentHeight, const double mountainHeight, const double radius, const double plateau, const double distance )
{
	//return segmentHeight + ( ( mountainHeight - segmentHeight ) * ( sin( ( ( cck::pi * ( 1.0 - ( ( distance - plateau ) / ( radius - plateau ) ) ) ) - cck::halfPi ) ) + 1.0 ) / 2.0 );
	return segmentHeight + ( 1.0 - ( ( distance - plateau ) / ( radius - plateau ) ) ) * ( mountainHeight - segmentHeight );
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

void cck::Globe::SampleData( const double sampleLatitude, const double sampleLongitude, double& sampleHeight, int& sampleId ) const
{
	SampleData( cck::GeoCoord( sampleLatitude * cck::pi / 180.0, sampleLongitude * cck::pi / 180.0 ), sampleHeight, sampleId );
}

void cck::Globe::SampleData( const cck::GeoCoord& sampleCoord, double& sampleHeight, int& sampleId ) const	//TODO: rename height in all functions to sampleHeight, and id to sampleId
{
	const cck::Vec3 samplePoint = sampleCoord.ToCartesian( globeRadius );
	const double noiseValue = noise.ScaledOctaveNoise( samplePoint.x, samplePoint.y, samplePoint.z, noiseOctaves, noisePersistance, noiseFrequency, 0.0, 1.0 );

	if ( noiseValue < seaScale )
	{
		sampleHeight = 0.0;
		sampleId = -1;
		return;
	}

	for ( const auto& triangle : triangles )
	{
		double tempHeight = 0.0;
		int tempId = -1;
		if ( triangle->SampleData( sampleCoord, samplePoint, globeRadius, tempHeight, tempId ) )
		{
			double influence = ( noiseValue - seaScale ) / ( 1.0 - seaScale );
			sampleHeight = tempHeight * influence;
			sampleId = tempId;
			return;
		}
	}


	double highestHeight = 0.0;
	int highestId = -1;

	for ( const auto& edge : edges )
	{
		double influence = edge->GetInfluence( sampleCoord, samplePoint, globeRadius ) * noiseValue;
		if ( influence >= seaScale )
		{
			influence = ( influence - seaScale ) / ( 1.0 - seaScale );
			double tempHeight = 0.0;
			int tempId = -1;
			edge->SampleData( sampleCoord, samplePoint, globeRadius, tempHeight, tempId );
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
		double influence = node->GetInfluence( sampleCoord, globeRadius ) * noiseValue;
		if ( influence >= seaScale )
		{
			influence = ( influence - seaScale ) / ( 1.0 - seaScale );
			double tempHeight = 0.0;
			int tempId = -1;
			node->SampleData( tempHeight, tempId );
			tempHeight *= influence;
			if ( tempHeight > highestHeight )
			{
				highestHeight = tempHeight;
				highestId = tempId;
			}
		}
	}

	sampleHeight = highestHeight;
	sampleId = highestId;
}

void cck::Globe::SampleInfluence( const double sampleLatitude, const double sampleLongitude, double& sampleInfluence ) const
{
	SampleInfluence( cck::GeoCoord( sampleLatitude * cck::pi / 180.0, sampleLongitude * cck::pi / 180.0 ), sampleInfluence );
}

void cck::Globe::SampleInfluence( const cck::GeoCoord& sampleCoord, double& sampleInfluence ) const
{
	sampleInfluence = 0.0;
	const cck::Vec3 samplePoint = sampleCoord.ToCartesian( globeRadius );

	for ( const auto& triangle : triangles )
	{
		double tempHeight = 0.0;
		int tempId = -1;
		if ( triangle->SampleData( sampleCoord, samplePoint, globeRadius, tempHeight, tempId ) )
		{
			sampleInfluence = 1.0;
			return;
		}
	}

	double greatestInfluence = 0.0;

	for ( const auto& edge : edges )
	{
		double influence = edge->GetInfluence( sampleCoord, samplePoint, globeRadius );
		if ( influence > greatestInfluence )
		{
			greatestInfluence = influence;
		}
	}

	for ( const auto& node : nodes )
	{
		double influence = node->GetInfluence( sampleCoord, globeRadius );
		if ( influence > greatestInfluence )
		{
			greatestInfluence = influence;
		}
	}

	sampleInfluence = greatestInfluence;
}

void cck::Globe::SetInfluenceFactor( const double newInfluenceFactor )
{
	influenceFactor = newInfluenceFactor;
}

cck::NoiseError	cck::Globe::SetNoiseParameters( const int octaves, const double persistance, const double frequency )
{
	if ( octaves <= 0 )
	{
		return cck::NoiseError::NON_POSITIVE_OCTAVES;
	}

	if ( persistance <= 0.0 )
	{
		return cck::NoiseError::NON_POSITIVE_PERSISTANCE;
	}

	if ( frequency <= 0.0 )
	{
		return cck::NoiseError::NON_POSITIVE_FREQUENCY;
	}

	noiseOctaves = octaves;
	noisePersistance = persistance;
	noiseFrequency = frequency;
	return cck::NoiseError::SUCCESS;
}

cck::Globe::Globe( const double globeRadius, const double seaScale, const unsigned int seed )
	:	globeRadius( globeRadius ),
		seaScale( ( seaScale < 0.0 || seaScale > 1.0 ) ? 0.4 : seaScale ),
		noise( seed ),
		noiseOctaves( 7 ),
		noisePersistance( 0.6 ),
		noiseFrequency( 0.0001 ),
		influenceFactor( 1.0 )
{
}
