#include "DegenerateVertexPatch.h"

#include "DuplicateVertices.h"

void defects::DegenerateVertexPatch::_detect(const MeshData& mesh)
{
	double sqrd_tolerance = tolerance * tolerance;
	const auto& vertices = mesh.get_vertices();
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		Eigen::RowVector3d vi = vertices.row(i);
		const auto& neighbourhood = mesh.get_adj_vertices(i);
		for (Eigen::Index j : neighbourhood)
		{
			if (j > i && (vertices.row(j) - vi).squaredNorm() <= sqrd_tolerance)
				vertex_clusters.add(i, j);
		}
	}
}

void defects::DegenerateVertexPatch::_repair(MeshData& mesh)
{
	remove_duplicate_vertices(mesh, vertex_clusters);
}

void defects::DegenerateVertexPatch::reset()
{
	vertex_clusters.clear();
}

bool defects::DegenerateVertexPatch::in_detected_state() const
{
	return !vertex_clusters.empty();
}
