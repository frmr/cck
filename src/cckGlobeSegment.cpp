#include "cckGlobe.h"

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
