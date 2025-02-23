#pragma once

#include "Common.h"

namespace defects
{
	struct NonManifoldEdges : public DefectBase
	{
	protected:
		virtual void _detect(const MeshData& mesh) override;
		virtual void _repair(MeshData& mesh) override;

	public:
		virtual void reset() override;
		virtual bool in_detected_state() const override;

	private:
		struct UndirectedEdge
		{
			Eigen::Index v1, v2;

			bool operator==(const UndirectedEdge& other) const { return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1); }
		};

		struct UndirectedEdgeHash
		{
			size_t operator()(const UndirectedEdge& edge) const { return std::hash<Eigen::Index>{}(edge.v1) ^ std::hash<Eigen::Index>{}(edge.v2); }
		};

		std::unordered_map<UndirectedEdge, size_t, UndirectedEdgeHash> edge_counts;
		std::unordered_set<UndirectedEdge, UndirectedEdgeHash> non_manifold_edges;
		void add_undirected_edge(Eigen::Index v1, Eigen::Index v2);

	public:
		void traverse_non_manifold_edges(const std::function<void(Eigen::Index v1, Eigen::Index v2, size_t count)>& process) const;
	};
}
