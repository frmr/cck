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
	public:
		enum class NodeType
		{
			LAND,
			SEA,
			RANDOM
		};

	private:

		class Node;

		class Link
		{
		private:
			NodeType					type;
			vector<shared_ptr<Node>>	nodes;

		public:
			bool LinksNode( const size_t &nodeId ) const;

		public:
			Link( const NodeType type, const shared_ptr<Node> nodeA, const shared_ptr<Node> nodeB );
		};

		class Node
		{
		public:
			bool LinkedToNode( const size_t &nodeId ) const;

		private:
			vector<shared_ptr<Link>>	links;

		public:
			const size_t				id;
			const cck::GeoCoord			coord;
			const cck::Vec3				position;
			const NodeType				type;
			const double				radius;


		public:
			void AddLink( const shared_ptr<Link> newLink );

		public:
			Node( const size_t &id, const cck::GeoCoord &coord, const Vec3 &position, const NodeType type, const double &radius );
		};

	private:
		double						radius;
		vector<shared_ptr<Node>>	nodes;


	private:
		double Distance( const GeoCoord &coordA, const GeoCoord &coordB )const;

	public:
		cck::LinkError	AddLink( const NodeType type, const size_t &nodeIdA, const size_t &nodeIdB );
		cck::NodeError	AddNode( const size_t &id, const double &latitude, const double &longitude, const NodeType type, const double &nodeRadius );
		cck::NodeError	AddNode( const size_t &id, const cck::GeoCoord &coord, const NodeType type, const double &nodeRadius );

		double			GetHeight( const double &latitude, const double &longitude ) const;
		double			GetHeight( const cck::GeoCoord &coord ) const;

		size_t			GetNodeId( const double &latitude, const double &longitude ) const;
		size_t			GetNodeId( const cck::GeoCoord &coord ) const;


	public:
		Globe( const int seed, const double &radius );
	};
}
#endif // CCK_GLOBE_H
