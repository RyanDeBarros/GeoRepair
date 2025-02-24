#include "UnboundVertices.h"

void defects::UnboundVertices::_detect(const Mesh& mesh)
{
	const auto& vertices = mesh.get_vertices();
	for (Eigen::Index i = 0; i < vertices.rows(); ++i)
	{
		const Eigen::RowVector3d& vertex = vertices.row(i);
		if (vertex(0) > max_x || vertex(0) < min_x || vertex(1) > max_y || vertex(1) < min_y || vertex(2) > max_z || vertex(2) < min_z)
			unbound_vertices.push_back(i);
	}
}

void defects::UnboundVertices::_repair(Mesh& mesh)
{
	auto& vertices = mesh.get_vertices();
	for (Eigen::Index v : unbound_vertices)
	{
		vertices(v, 0) = std::clamp(vertices(v, 0), min_x, max_x);
		vertices(v, 1) = std::clamp(vertices(v, 1), min_y, max_y);
		vertices(v, 2) = std::clamp(vertices(v, 2), min_z, max_z);
	}
}

void defects::UnboundVertices::reset()
{
	unbound_vertices.clear();
}

bool defects::UnboundVertices::in_detected_state() const
{
	return !unbound_vertices.empty();
}
