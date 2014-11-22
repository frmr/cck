#include "cckGlobe.h"
#include "cckMath.h"

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
