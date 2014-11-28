#include "cckGlobe.h"

#include <limits>

void cck::Globe::Segment::AddEdge( const shared_ptr<Edge>& newEdge )
{
	mountainEdges.push_back( newEdge );
}

void cck::Globe::Segment::AddNode( const shared_ptr<Node>& newNode )
{
	mountainNodes.push_back( newNode );
}

void cck::Globe::Segment::SampleData( const cck::GeoCoord& sampleCoord, const cck::Vec3& samplePoint, const double globeRadius, const double noiseValue, double& sampleHeight, int& sampleId ) const
{
	sampleId = baseNode->id;
	const double segmentHeight = baseNode->minHeight + noiseValue * ( baseNode->maxHeight - baseNode->minHeight );

	double highestMountain = std::numeric_limits<double>::min();
	bool foundMountain = false;

	for ( const auto& edge : mountainEdges )
	{
		if ( edge->IsActive() )
		{
			const double mountainHeight = edge->GetMountainHeight( sampleCoord, samplePoint, globeRadius, noiseValue, segmentHeight );

			if ( mountainHeight > highestMountain )
			{
				highestMountain = mountainHeight;
				foundMountain = true;
			}
		}
	}

	if ( !foundMountain )
	{
		for ( const auto& node : mountainNodes )
		{
			const double mountainHeight = node->GetMountainHeight( sampleCoord, globeRadius, noiseValue, segmentHeight );

			if ( mountainHeight > highestMountain )
			{
				highestMountain = mountainHeight;
				foundMountain = true;
			}
		}
	}

	sampleHeight = ( foundMountain ) ? highestMountain : segmentHeight;
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
