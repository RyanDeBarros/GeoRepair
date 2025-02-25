#pragma once

#include "Common.h"

namespace defects
{
	struct NonManifoldEdges : public DefectBase
	{
		virtual void _detect(const Mesh& mesh) override;
		virtual void _repair(Mesh& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

		struct UndirectedEdge
		{
			Eigen::Index v1, v2;

			bool operator==(const UndirectedEdge& other) const { return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1); }
		};

	private:
		struct UndirectedEdgeHash
		{
			size_t operator()(const UndirectedEdge& edge) const { return std::hash<Eigen::Index>{}(edge.v1) ^ std::hash<Eigen::Index>{}(edge.v2); }
		};

		std::unordered_map<UndirectedEdge, size_t, UndirectedEdgeHash> edge_counts;
		std::unordered_set<UndirectedEdge, UndirectedEdgeHash> non_manifold_edges;
		int max_edge_count = 0;
		void add_undirected_edge(Eigen::Index v1, Eigen::Index v2);

	public:
		const decltype(non_manifold_edges)& get_non_manifold_edges() const { return non_manifold_edges; }
		const decltype(edge_counts)& get_edge_counts() const { return edge_counts; }
		int get_max_edge_count() const { return max_edge_count; }
	};
}
