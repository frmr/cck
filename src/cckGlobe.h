#ifndef CCK_GLOBE_H
#define CCK_GLOBE_H

#include <memory>
#include <vector>
#include <queue>

#include "cckError.h"
#include "cckGeoCoord.h"
#include "cckSimplexNoise.h"
#include "cckVec3.h"

using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace cck
{
	class Globe
	{
	private:

		class Node;
		class Side;
		class Edge;
		class Segment;




		class BspTree
		{
		private:

			class BspNode
			{
			private:
				shared_ptr<Edge>	edge;
				shared_ptr<Segment>	segment;
				shared_ptr<BspNode>	posChild; //change these two to unique_ptr in C++14
				shared_ptr<BspNode>	negChild;

			public:
				bool				AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& edge );
				bool				AddSegment( std::queue<bool>& coord, const shared_ptr<Segment>& segment );
				shared_ptr<Segment>	GetSegment( const cck::Vec3& point ) const;
				bool				IsComplete() const;

			public:
				BspNode();
			};

		private:
			BspNode root;

		public:
			bool				AddChildren( std::queue<bool>& coord, const shared_ptr<Edge>& edge );
			bool				AddSegment( std::queue<bool>& coord, const shared_ptr<Segment>& segment );
			shared_ptr<Segment>	GetSegment( const cck::Vec3& point ) const;
			bool				IsComplete() const;

		public:
			BspTree();
		};




		class Edge: public std::enable_shared_from_this<Edge>
		{
		public:
			const shared_ptr<Node>		nodeA;
			const shared_ptr<Node>		nodeB;
			const double				length;
			const shared_ptr<Node>		centerNode;		//nullptr if a mountain Edge
			const cck::Vec3				normal;
			const BspTree				tree;
			vector<shared_ptr<Side>>	sides;			//this should probably be private

		private:
			cck::Vec3					ClosestPoint( const cck::Vec3& point ) const;
			BspTree						ConstructTree( const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius ) const;
			bool						Contains( const cck::Vec3& point ) const;
			bool						PointOnFreeSide( const cck::Vec3& point ) const;

		public:
			void						AddSides();
			void						GetContinentData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;
			void 						GetMountainData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;
			//double						GetInfluence( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius ) const; //remove

		public:
			Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double mountainHeight, const double mountainRadius, const double mountainPlateau, const double globeRadius );
			Edge( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const double globeRadius );
		};





		class Side
		{
		private:
			bool formsTriangle;

		public:
			const cck::Vec3			normal;
			const shared_ptr<Edge>	edge;
			//TODO: Add Triangle variable, can only have one

		public:
			bool FormsTriangle() const;
			void SetFormsTriangle();

		public:
			Side( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Edge>& edge );
		};




		class Link
		{
		public:
			const shared_ptr<Node>	target;
			const shared_ptr<Edge>	edge;

		public:
			bool LinksTo( const int nodeId ) const;

		public:
			Link( const shared_ptr<Node>& target, const shared_ptr<Edge>& edge );
		};




		class Node
		{
		private:
			vector<shared_ptr<Link>>	links;

		public:
			const int					id;
			const cck::GeoCoord			coord;
			const cck::Vec3				position;
			const cck::Vec3				unitVec;
			const double				height;
			const double				radius;
			const double				plateau;

		public:
			void 						AddLink( const shared_ptr<Link>& newLink );
			vector<shared_ptr<Node>>	FindCommonNeighbors( const shared_ptr<Node>& refNode );
			void						GetContinentData( const cck::GeoCoord &pointCoord, const double globeRadius, double& sampleHeight, int& sampleId ) const;
			shared_ptr<Link>			GetLinkTo( const int targetId ) const;
			void						GetMountainData( const cck::GeoCoord &pointCoord, const double globeRadius, double& sampleHeight, int& sampleId ) const;
			bool						LinkedTo( const int nodeId ) const;

		public:
			Node( const int id, const cck::GeoCoord& coord, const double height, const double radius, const double globeRadius );			//Node for continent definitions
			Node( const cck::Vec3& position, const double height, const double radius, const double plateau );	//Node for mountain definitions
		};





		class Triangle
		{
		private:
			const vector<shared_ptr<Node>>	nodes;
			const Node						centerNode;
			const vector<shared_ptr<Side>>	sides;
			const BspTree					tree;
			//TODO: Bounding box class

		private:
			BspTree						ConstructTree() const;
			bool 						Contains( const cck::Vec3& point ) const;
			Node						CreateCenterNode() const;
			vector<shared_ptr<Node>>	CreateNodeVector( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC ) const;

		public:
			void						GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;
			//int		GetNodeId( const cck::GeoCoord& coord, const double globeRadius ) const;


		public:
			Triangle( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC, const vector<shared_ptr<Side>>& sides );
		};




		class Segment
		{
		private:
			const shared_ptr<Node>				baseNode;
			const vector<shared_ptr<Node>>		mountainNodes;
			const vector<shared_ptr<Edge>>		mountainEdges;

		public:
			void GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;

		public:
			Segment( const shared_ptr<Node>& baseNode, const vector<shared_ptr<Node>>& mountainNodes, const vector<shared_ptr<Edge>>& mountainEdges );
		};




	private:
		double							globeRadius;
		vector<shared_ptr<Node>>		nodes;
		vector<shared_ptr<Edge>>		edges;
		vector<shared_ptr<Triangle>>	triangles;
		const cck::SimplexNoise			simplex;

	private:
		void			GetHeight( const cck::GeoCoord& coord, double& height ) const;
		int				GetNodeId( const cck::GeoCoord& coord ) const;

	public:
		cck::NodeError	AddNode( const int id, const double latitude, const double longitude, const double height, const double nodeRadius );
		cck::NodeError	AddNode( const int id, const cck::GeoCoord& coord, const double height, const double nodeRadius );

		//shared_ptr<Segment>	CreateLineSegment( const shared_ptr<Node>& baseNode)

		cck::LinkError	LinkNodes( const int nodeIdA, const int nodeIdB, const double mountainHeight, const double mountainRadius, const double mountainPlateau );

		void			GetData( const double latitude, const double longitude, double& height, int& id ) const;
		void			GetData( const cck::GeoCoord& coord, double& height, int& id ) const;

	public:
		Globe( const double radius, const unsigned int seed );
	};
}
#endif // CCK_GLOBE_H
