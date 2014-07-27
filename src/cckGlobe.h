#ifndef CCK_GLOBE_H
#define CCK_GLOBE_H

#include <memory>
#include <vector>

#include "cckGeoCoord.h"
#include "cckError.h"
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
		private:
			vector<shared_ptr<Side>>	sides;

		public:
			const shared_ptr<Node>		nodeA;
			const shared_ptr<Node>		nodeB;
			const double				borderScale;

		public:
			void		AddSides();
			cck::Vec3	GetNormal() const;

		public:
			Edge( const shared_ptr<Node> &nodeA, const shared_ptr<Node> &nodeB, const double borderScale );
		};




		class Side
		{
		public:
			const cck::Vec3			normal;
			const shared_ptr<Edge>	edge;
			//TODO: Add Triangle variable, can only have one

		public:
			Side( const shared_ptr<Node> &nodeA, const shared_ptr<Node> &nodeB, const shared_ptr<Edge> &edge );
		};




		class Link
		{
		public:
			const shared_ptr<Node>	target;
			const shared_ptr<Edge>	edge;

		public:
			bool LinksTo( const size_t &nodeId ) const;

		public:
			Link( const shared_ptr<Node> &target, const shared_ptr<Edge> &edge );
		};




		class Node
		{
		private:
			vector<shared_ptr<Link>>	links;

		public:
			bool						LinkedTo( const size_t &nodeId ) const;
			vector<shared_ptr<Node>>	FindCommonNeighbors( const shared_ptr<Node> &refNode );

		public:
			const size_t				id;
			const cck::GeoCoord			coord;
			const cck::Vec3				position;
			const double				radius;

		public:
			void AddLink( const shared_ptr<Link> &newLink );

		public:
			Node( const size_t &id, const cck::GeoCoord &coord, const Vec3 &position, const double &radius );
		};




		class Triangle
		{
		private:
			vector<shared_ptr<Node>>	nodes;
			vector<shared_ptr<Side>>	sides;
			//TODO: Bounding box class

		public:
			bool 	Contains( const cck::GeoCoord &coord ) const;
			double	GetHeight( const cck::GeoCoord &coord ) const;


		public:
			//Triangle( const vector<shared_ptr<Node>> &nodes, const vector<shared_ptr<Edge> &edges );
			Triangle( const shared_ptr<Node> &nodeA, const shared_ptr<Node> &nodeB, const shared_ptr<Node> &nodeC, const vector<shared_ptr<Edge>> &edges );

		};


	private:
		double							globeRadius;
		vector<shared_ptr<Node>>		nodes;
		vector<shared_ptr<Edge>>		edges;
		vector<shared_ptr<Triangle>>	triangles;

	private:
		double Distance( const GeoCoord &coordA, const GeoCoord &coordB ) const;

	public:
		cck::LinkError	LinkNodes( const size_t &nodeIdA, const size_t &nodeIdB, const double borderScale );
		cck::NodeError	AddNode( const size_t &id, const double &latitude, const double &longitude, const double &nodeRadius );
		cck::NodeError	AddNode( const size_t &id, const cck::GeoCoord &coord, const double &nodeRadius );

		double			GetHeight( const double &latitude, const double &longitude ) const;
		double			GetHeight( const cck::GeoCoord &coord ) const;

		size_t			GetNodeId( const double &latitude, const double &longitude ) const;
		size_t			GetNodeId( const cck::GeoCoord &coord ) const;


	public:
		Globe( const int seed, const double &radius );
	};
}
#endif // CCK_GLOBE_H
