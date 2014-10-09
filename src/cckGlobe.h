#ifndef CCK_GLOBE_H
#define CCK_GLOBE_H

#include <memory>
#include <vector>

#include "cckError.h"
#include "cckGeoCoord.h"
#include "cckSimplexNoise.h"
#include "cckVec3.h"

using std::shared_ptr;
using std::vector;

namespace cck
{
	class Globe
	{
	private:

		class Node;
		class Side;



		class Edge: public std::enable_shared_from_this<Edge>
		{
		public:
			const shared_ptr<Node>		nodeA;
			const shared_ptr<Node>		nodeB;
			const double				length;
			const shared_ptr<Node>		centerNode; //nullptr if a mountain Edge
			const cck::Vec3				normal;
			vector<shared_ptr<Side>>	sides; //this should probably be private

		private:
			cck::Vec3					ClosestPoint( const cck::Vec3& point ) const;
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
			void						GetData( const cck::GeoCoord &pointCoord, const double globeRadius, double& sampleHeight, int& sampleId ) const;
			shared_ptr<Link>			GetLinkTo( const int targetId ) const;
			bool						LinkedTo( const int nodeId ) const;

		public:
			Node( const int id, const cck::GeoCoord& coord, const double globeRadius, const double height, const double radius );			//Node for continent definitions
			Node( const cck::GeoCoord& coord, const double globeRadius, const double height, const double radius, const double plateau );	//Node for mountain definitions
		};





		class Triangle
		{
		private:
			cck::Vec3					midpoint;
			vector<shared_ptr<Node>>	nodes;
			vector<shared_ptr<Side>>	sides; 
			//TODO: Bounding box class

		private:
			bool 	Contains( const cck::Vec3& point ) const;

		public:
			void	GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;
			//int		GetNodeId( const cck::GeoCoord& coord, const double globeRadius ) const;


		public:
			Triangle( const shared_ptr<Node>& nodeA, const shared_ptr<Node>& nodeB, const shared_ptr<Node>& nodeC, const vector<shared_ptr<Side>>& sides );

		};




		class Section
		{
		private:
			const shared_ptr<Node>				baseNode;
			const vector<shared_ptr<Node>>		mountainNodes;
			const vector<shared_ptr<Edge>>		mountainEdges;

		public:
			void GetData( const cck::GeoCoord& coord, const cck::Vec3& point, const double globeRadius, double& height, int& id ) const;

		public:
			Section( const shared_ptr<Node>& baseNode, const vector<shared_ptr<Node>>& mountainNodes, const vector<shared_ptr<Edge>>& mountainEdges );
		};




		class BspTree
		{
		private:
			
			class BspNode
			{
			protected:
				const shared_ptr<Edge> edge;

			protected:
				virtual shared_ptr<BspNode> GetChild( const cck::Vec3& point ) const;

			protected:
				BspNode( const shared_ptr<Edge>& edge );
			};


			class BspInternalNode : public BspNode
			{
			private:
				const shared_ptr<BspNode> posNode;
				const shared_ptr<BspNode> negNode;

			public:
				bool AddChildren( const shared_ptr<BspNode>& posNode, const shared_ptr<BspNode>& negNode );
				shared_ptr<BspNode> GetChild( const cck::Vec3& point ) const;

			public:
				BspInternalNode( const shared_ptr<Edge>& edge, const shared_ptr<BspNode>& posNode, const shared_ptr<BspNode>& negNode );
			};


			class BspLeafNode : public BspNode
			{
			private:
				const shared_ptr<Section> posSection;
				const shared_ptr<Section> negSection;

			public:
				shared_ptr<BspNode> GetChild( const cck::Vec3& point ) const;

			public:
				BspLeafNode( const shared_ptr<Edge>& edge, const Section& posSection, const Section& negSection );
				
			};

		public:
			shared_ptr<Section> GetSection( const cck::Vec3& point ) const;

		public:
			BspTree( 



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

		cck::LinkError	LinkNodes( const int nodeIdA, const int nodeIdB, const double mountainHeight, const double mountainRadius, const double mountainPlateau );

		void			GetData( const double latitude, const double longitude, double& height, int& id ) const;
		void			GetData( const cck::GeoCoord& coord, double& height, int& id ) const;

	public:
		Globe( const double radius, const unsigned int seed );
	};
}
#endif // CCK_GLOBE_H
